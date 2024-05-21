%% Optimal task allocation along a given path with genetic optimization
% assuming path{1} ~= robot.node

function [actions, fval, analysis] = ga_task_allocator(robot, path)
    % caching with a digraph
    cache = digraph(0, ...
                    table("0", 0, seconds(0), robot.energy, {""}, {[]}, false, 0, 0, ...
                   'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag', 'children', 'depth'}));
    rf = rowfilter(["path_idx", "action"]);
    rf_mutation = rowfilter(["depth", "flag", "children"]);
    % preprocess the tasks that can be done along the way
    [candidates, time_loss, energy_loss] = preprocess_path(robot, path);

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
                    flag = flag | u == u_prev;
                end
                % aggregrate the utility of closer points on the path
                % u = u + u_prev;
                % cache the next state
                cache = cache.addnode(table(next_state, u, t, e, {tau}, {u_tau}, flag, 0, j, ...
                     'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag', 'children', 'depth'}));
                cache = cache.addedge(state, next_state, 1);
                cache.Nodes.children(idx) = cache.Nodes.children(idx) + 1;
                state = next_state;
                % if flagged, no need to continue
                if flag && j < length(x)
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

    % plots = {'gaplotscores', ...
    %          'gaplotdistance', ...
    %          'gaplotbestf', ...
    %          'gaplotselection'};
    plots = {};

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
                   3*ones(size(path)), ... % up
                   [], ... % nonlcon
                   1:length(path), ... % intcon
                   options);
    actions = robot.policy.configs.action_list(x);
    fval = -fval;

    cache = rmnode(cache, cache.Nodes.Name(cache.Nodes.flag));
    idx = find(cellfun(@(y) strlength(y)==length(path)+1, cache.Nodes.Name));
    
    % analysis data
    analysis.cache = cache;
    analysis.cache_idx = idx;
    analysis.utility = -cache.Nodes(rf_mutation.depth == length(path), :).u';
    
    % calculate cache fullness
    cache_length = sum(cellfun(@(y) numel(y)==length(x)+1, cache.Nodes.Name));
    ratio = cache_length / length(robot.policy.configs.action_list)^length(x);
    analysis.log = sprintf('%s |fval: %.3f |size: %d |ratio: %.3f', ...
        robot.id, ...
        fval, ...
        cache_length, ...
        ratio);
end

