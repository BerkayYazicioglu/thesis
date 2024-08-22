%% Optimal task allocation along a given path with brute force optimization
% assuming path{1} ~= robot.node

function [trajectory, fval, analysis] = brute_force_task_allocator(robot, charger, path)
    % caching with a digraph
    cache = digraph(0, ...
                    table("0", 0, seconds(0), robot.energy, {[""]}, {[]}, 0, ...
                   'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag'}));
    rf = rowfilter(["path_idx", "action"]);
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
                % if flagged, the branch is pruned
                if cache.Nodes.flag(idx) == 1
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
                cache = cache.addnode(table(next_state, u, t, e, {tau}, {u_tau}, flag, ...
                                     'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag'}));
                cache = cache.addedge(state, next_state, 1);
                state = next_state;
                % if flagged, no need to continue
                if flag == 1
                    break
                end
            end
        end
    end

    %% Optimize the path task allocation using the fitness function
    % traverse the graph breadth-first and with pruning
    leaf_nodes = {"0"};
    leaf_idx = 1;
    traverse = true;
    while traverse
        % expand from the leaf node
        for ii = 1:length(robot.policy.configs.action_list)
            new_node = leaf_nodes{leaf_idx} + string(ii);
            x = str2num(char(num2cell(char(new_node))));
            fitness(x(2:end));
            % prune flagged children
            pruned_idx = findnode(cache, new_node);
            if pruned_idx
                if cache.Nodes.flag(pruned_idx) ~= 1
                    leaf_nodes = [leaf_nodes new_node];
                end
            end
        end
        leaf_nodes(leaf_idx) = [];
        % Find the index of the shortest string
        if isempty(leaf_nodes)
            break
        end
        [~, leaf_idx] = min(cellfun(@strlength, leaf_nodes));
        depth = strlength(leaf_nodes{leaf_idx});
        traverse = depth <= length(path);
    end

    cache.Nodes.flag(1) = 0;
    n_func = height(cache.Nodes.Name);
    % get final utilities
    cache = rmnode(cache, cache.Nodes.Name(cache.Nodes.flag == 1));
    max_length = max([arrayfun(@(x) strlength(x), cache.Nodes.Name)]);
    end_flags = [arrayfun(@(y) strlength(y) == max_length, cache.Nodes.Name)];
    const_flags = [arrayfun(@(y) y == 2, cache.Nodes.flag)];
    idx = find(end_flags | const_flags);
    
    % analysis data
    analysis.n_func = n_func;
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
        "brute_force");
end


