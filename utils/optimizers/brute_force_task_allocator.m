%% Optimal task allocation along a given path with brute force optimization
% assuming path{1} ~= robot.node

function [trajectory, fval, analysis] = brute_force_task_allocator(robot, charger, path)
    % caching with a digraph
    cache = digraph(0, ...
                    table("0", 0, seconds(0), robot.energy, {[""]}, {[]}, false, ...
                   'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag'}));
    rf = rowfilter(["path_idx", "action"]);
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
                % if flagged, the branch is pruned
                if cache.Nodes.flag(idx)
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

                % check charging constraints
                const_flag = charge_constraints(charger, robot, path(j), e, t);
                if ~const_flag && ~flag
                    flag = 2;
                end
                % cache the next state
                cache = cache.addnode(table(next_state, u, t, e, {tau}, {u_tau}, flag, ...
                                     'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'u_tau', 'flag'}));
                cache = cache.addedge(state, next_state, 1);
                state = next_state;
                % if flagged, no need to continue
                if flag
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
            if ~cache.Nodes.flag(findnode(cache, new_node))
                leaf_nodes = [leaf_nodes new_node];
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

    % get final utilities
    cache = rmnode(cache, cache.Nodes.Name(cache.Nodes.flag == 1));

    end_flags = [arrayfun(@(y) strlength(y) == length(path)+1, cache.Nodes.Name)];
    const_flags = [arrayfun(@(y) y == 2, cache.Nodes.flag)];
    idx = find(end_flags | const_flags);
    fval = 0;
    actions = robot.policy.configs.action_list(ones(1, length(path)));
    actions = [actions{:}];
    times = seconds([arrayfun(@(x) sum(time_loss(1:x)), 1:length(time_loss))]);
    tasks = [arrayfun(@(x) {}, 1:length(path), 'UniformOutput', false)]';

    % analysis data
    analysis.cache = cache;
    analysis.cache_idx = idx;
    analysis.utility = zeros(1, length(idx));
    analysis.log = "";

    for ii = 1:length(idx)
        x = cache.Nodes.Name(idx(ii));
        u = cache.Nodes.u(idx(ii));
        flag = cache.Nodes.flag(idx(ii));
        if u <= fval
            fval = u;
            t_idx = cache.findnode(cache.shortestpath('0', x));
            times = seconds(cache.Nodes.t(t_idx(2:end)));
            tasks = cache.Nodes.tau(t_idx(2:end));
            x = str2num(char(num2cell(char(x))));
            actions = robot.policy.configs.action_list(x(2:end));
            if flag == 2
                actions = [actions {"charge"}];
            end
            actions = [actions{:}];
        end
        analysis.utility(ii) = -u; 
    end
    
    if actions(end) == "charge"
        actions(end) = [];
        [~, idx] = min(abs(charger.schedule.Time - times(end)));
        charger_node = num2str(charger.schedule.node(idx));
        % find the shortest path to the charger node
        [p, ~, edge] = robot.map.shortestpath(path(length(actions)), charger_node);
        last_t = times(end);
        path = path(1:length(actions));
        for i = 2:length(p)
            distance = robot.world.environment.Edges(edge(i-1),:).Weight;
            times = [times(:); last_t + distance/robot.speed];
            actions = [actions(:); "none"];
            path = [path(:); p{i}];
            last_t = times(end);
            tasks = [tasks(:); {""}];
        end
        actions(end) = "charge";
        times(end) = times(end) + seconds(robot.charge_s);
    end
    times = seconds(times);
    fval = -fval;

    trajectory = table(times(:), path(:), actions(:), tasks, ...
        'VariableNames', {'time', 'node', 'action', 'tasks'});
    

    % calculate cache fullness
    cache_length = sum(cellfun(@(y) numel(y)>=length(path)+1, cache.Nodes.Name));
    ratio = cache_length / length(robot.policy.configs.action_list)^length(path);
    analysis.log = sprintf('%s |fval: %.3f |size: %d |ratio: %.3f', ...
        robot.id, ...
        fval, ...
        cache_length, ...
        ratio);
end


