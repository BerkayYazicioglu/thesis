%% Optimal task allocation along a given path with genetic optimization
% assuming path{1} ~= robot.node

function [trajectory, fval, analysis] = ga_task_allocator(robot, charger, path)
    % caching with a digraph
    cache = digraph(0, ...
                    table("0", 0, seconds(0), robot.energy, {""}, {[]}, 0, 0, 0, ...
                   'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag', 'children', 'depth'}));
    rf = rowfilter(["path_idx", "action"]);
    rf_mutation = rowfilter(["depth", "flag", "children"]);
    % preprocess the tasks that can be done along the way
    [candidates, ...
     time_loss, ... 
     energy_loss, ...
     analysis.minima_flag] = preprocess_path(robot, path);

    %% Fitness function
    % 1 -> no action
    % 2 -> explore
    % 3 -> search
    % 4 -> both                         
    function u = fitness(x)
        state = "0";
        for j = 1:length(x)
            next_state = state + string(x(j));
            % check if the current state transition is cached
            children = cache.successors(state);
            if ismember(next_state, children)
                % next state is already cached
                state = next_state;
                idx = cache.findnode(state);
                u = cache.Nodes.u(idx);
                t = cache.Nodes.t(idx);
                e = cache.Nodes.e(idx);
                flag = cache.Nodes.flag(idx);
                % if flagged, the branch is pruned
                if flag
                    final_state = "0" + erase(num2str(x), ' ');
                    if ~findnode(cache, final_state)
                        cache = cache.addnode(table(final_state, 0, t, e, {""}, {[]}, flag, 0, numel(x), ...
                            'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag', 'children', 'depth'}));
                        cache = cache.addedge(state, final_state, 1);
                    end
                    break
                end
            else
                % the state needs to be calculated and cached
                idx = cache.findnode(state);
                u = cache.Nodes.u(idx);
                t = cache.Nodes.t(idx);
                e = cache.Nodes.e(idx);
                tau = cache.Nodes.tau(idx);
                u_tau = cache.Nodes.u_tau(idx);
                flag = cache.Nodes.flag(idx);
                tau = tau{1};
                u_tau = u_tau{1};

                % movement
                t = t + seconds(time_loss(j));
                e = e - energy_loss(j);

                % actions
                action = robot.policy.configs.action_list{x(j)};
                u_prev = u;
                if action ~= "none" 
                    if action == "explore"
                        t = t + robot.visible_sensor.t_s;
                        e = e - robot.visible_sensor.d_energy;
                    elseif action == "search"
                        t = t + robot.remote_sensor.t_s;
                        e = e - robot.remote_sensor.d_energy;
                    elseif action == "both"
                        t = t + max(robot.visible_sensor.t_s, ...
                                    robot.remote_sensor.t_s);
                        e = e - robot.visible_sensor.d_energy ...
                              - robot.remote_sensor.d_energy;
                        action = ["search" "explore"];
                    end
                   
                    % get capabilities and weights of applicable tasks
                    for a = 1:length(action)
                        sublist = candidates(rf.path_idx == j & rf.action == action(a), :);
                        [performed, p_idx] = setdiff(sublist.task_node, tau);
                        if ~isempty(p_idx)
                            sublist_u = zeros(1, length(performed));
                            for k = 1:length(performed)
                                sublist_u(k) = sublist.u(p_idx(k));
                                u = u - sublist_u(k) / seconds(t);
                            end
                            tau = [tau; performed(:)];
                            u_tau = [u_tau; sublist_u(:)];
                        % passivity
                        elseif action(a) == "search"
                            break
                        end
                    end
                    % check if there was any improvement
                    if u == u_prev
                        flag = 1;
                    end
                end
                % aggregrate the utility of closer points on the path
                % u = u + u_prev;

                % check charging constraints
                const_flag = charge_constraints(charger, robot, path(j), e, t + robot.time);
                if ~const_flag 
                    % next state violates the constraint, flag current
                    % state
                    cache.Nodes.flag(idx) = 2;
                    flag = 1;
                end

                % cache the next state
                cache = cache.addnode(table(next_state, u, t, e, {tau}, {u_tau}, flag, 0, j, ...
                     'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag', 'children', 'depth'}));
                cache = cache.addedge(state, next_state, 1);
                cache.Nodes.children(idx) = cache.Nodes.children(idx) + 1;
                state = next_state;
                % if flagged, no need to continue
                if flag == 1 && j < length(x)
                    final_state = "0" + erase(num2str(x), ' ');
                    cache = cache.addnode(table(final_state, 0, t, e, {""}, {[]}, flag, 0, numel(x), ...
                        'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag', 'children', 'depth'}));
                    cache = cache.addedge(state, final_state, 1);
                    break
                end
            end
        end
    end

    %% Mutation function
    % parents — Row vector of parents chosen by the selection function
    % options — Options
    % nvars — Number of variables
    % FitnessFcn — Fitness function
    % state — Structure containing information about the current generation.
    %         The State Structure describes the fields of state.
    % thisScore — Vector of scores of the current population
    % thisPopulation — Matrix of individuals in the current population
    % 
    % The function returns mutationChildren—the mutated offspring—as a 
    % matrix where rows correspond to the children. 
    % The number of columns of the matrix is nvars.
    function children = mutation_heuristic(parents, ...
                                           options, ...
                                           nvars, ...
                                           FitnessFcn, ...
                                           state, ...
                                           thisScore, ...
                                           thisPopulation)
        action_nums = 1:length(robot.policy.configs.action_list);
        children = zeros(numel(parents), nvars);
        % find admissable nodes
        subcache = cache.Nodes(...
            rf_mutation.flag == false & ...
            rf_mutation.children < length(robot.policy.configs.action_list) & ...
            rf_mutation.depth < nvars,:);
        % no admissable nodes
        if isempty(subcache)
            children = randi(numel(action_nums), numel(parents), nvars);
            return
        end
        % convert parents to truncated strings
        parents_str = cellstr(num2str([zeros(size(parents))' thisPopulation(parents, :)], '%d'));

        % edit distance search
        % eds = editDistanceSearcher(subcache.Name, nvars*2);
        % children_str = eds.Vocabulary(knnsearch(eds, parents_str));
        
        % shortest path search
        search_graph = graph(cache.adjacency + cache.adjacency');
        sps = @(x) min(distances(search_graph, ...
                                 x, ...
                                 cache.findnode(subcache.Name)));
        [~, mut_idx] = arrayfun(sps, cache.findnode(parents_str));
        mut_str = subcache.Name(mut_idx);
        for i = 1:length(mut_str)
            mut_children = cache.successors(mut_str{i});
            mut = [cellfun(@(x)str2double(x), num2cell(mut_str{i}))];
            pruned = cellfun(@(x)str2double(x(end)), mut_children);
            valid = setdiff(action_nums, pruned);
            mut = [mut valid(randi(numel(valid)))];
            % create a random string to pad the remaining 
            mut = [mut randi(numel(action_nums), 1, nvars+1-length(mut))];
            children(i, :) = mut(2:end);
        end
    end

    %% Optimize the path task allocation using the fitness function
    popsize = 10;
    co_fraction = 0.4;
    sel_ratio = 0.2;
    elite_ratio = 0.1;
    max_stall = 10;

    plots = {};
    if robot.world.params.detail_plots
        plots = {'gaplotscores', ...
                 'gaplotdistance', ...
                 'gaplotbestf', ...
                 'gaplotselection'};
    end

    options = optimoptions('ga', ...
                           'PopulationSize', popsize, ...
                           'SelectionFcn', {@selectiontournament, ...
                                            ceil(sel_ratio * popsize)}, ...
                           'CrossoverFraction', co_fraction, ...
                           'EliteCount', ceil(elite_ratio * popsize), ...
                           'MutationFcn', @mutation_heuristic, ...
                           'CrossoverFcn', 'crossoverscattered', ...
                           'MaxStallGenerations', max_stall, ...
                           'Display', 'iter', ...
                           'PlotFcn', plots);
    [x, fval, ~, output] = ...
                ga(@fitness, ...
                   length(path), ... % numvars
                   [],[],[],[], ...  % A, b, Aeq, Beq 
                   ones(size(path)), ... % lb
                   length(robot.policy.configs.action_list)*ones(size(path)), ... % up
                   [], ... % nonlcon
                   1:length(path), ... % intcon
                   options);

    % get final utilities
    cache.Nodes.flag(1) = 0;
    cache = rmnode(cache, cache.Nodes.Name(cache.Nodes.flag == 1));
    max_length = max([arrayfun(@(x) strlength(x), cache.Nodes.Name)]);
    end_flags = [arrayfun(@(y) strlength(y) == max_length, cache.Nodes.Name)];
    const_flags = [arrayfun(@(y) y == 2, cache.Nodes.flag)];
    idx = find(end_flags | const_flags);

    % analysis data
    analysis.n_func = output.funccount;
    analysis.candidates = candidates;
    analysis.cache = cache;
    analysis.cache_idx = idx;
    analysis.utility = -cache.Nodes.u(idx)';
    analysis.log = "";

    [fval, best_idx] = max(analysis.utility);
     % passivity
    if sum(fval == analysis.utility) > 1 
        [~, best_idx] = min(cache.Nodes.t(idx(analysis.utility == fval)));
    end
    best_idx = idx(best_idx);

    x = cache.Nodes.Name(best_idx);
    t_idx = cache.findnode(cache.shortestpath('0', x));
    times = cache.Nodes.t(t_idx); % including origin
    tasks = cache.Nodes.tau(t_idx); % including origin
    x = str2num(char(num2cell(char(x))));
    actions = robot.policy.configs.action_list(x(2:end)); % excluding origin

    if isempty(actions)
        % nowhere to go
        actions = {"charge"};
    elseif cache.Nodes.flag(best_idx) == 2
        actions = [actions {"charge"}];
    end
    actions = [actions{:}];
    
    if actions(end) == "charge"
        actions(end) = [];
        [~, idx] = min(abs(charger.schedule.Time - times(end) - robot.time));
        charger_node = num2str(charger.schedule.node(idx));
        % find the shortest path to the charger node
        if ~isempty(actions)
            path = path(1:length(actions));
            [p, ~, edge] = robot.map.shortestpath(path(end), charger_node);
        else
            path = {};
            [p, ~, edge] = robot.map.shortestpath(robot.node, charger_node);
        end
        last_t = times(end);
        for i = 2:length(p)
            distance = robot.world.environment.Edges(edge(i-1),:).Weight;
            times = [times(:); last_t + seconds(distance/robot.speed)];
            actions = [actions(:); "none"];
            path = [path(:); p{i}];
            last_t = times(end);
            tasks = [tasks(:); {""}];
        end
        actions(end) = "charge";
        times(end) = times(end) + robot.charge_s;
    end
    times = times(2:end);
    tasks = tasks(2:end);
    % separate planned tasks
    d_tasks = cell(size(tasks));
    for i = 2:length(d_tasks)
        d_tasks{i} = setdiff(tasks{i}, tasks{i-1});
        if isempty(d_tasks{i})
            d_tasks{i} = "";
        end
    end
    d_tasks{1} = tasks{1};

    trajectory = table(times(:), path(:), actions(:), d_tasks, ...
        'VariableNames', {'time', 'node', 'action', 'tasks'});
    
    % calculate cache fullness
    cache_length = numel(analysis.cache_idx);
    analysis.ratio = cache_length / length(robot.policy.configs.action_list)^length(path);
    analysis.log = sprintf('%s |fval: %.3f |size: %d |ratio: %.3f |optimizer: %s', ...
        robot.id, ...
        fval, ...
        cache_length, ...
        analysis.ratio, ...
        "ga");
end

