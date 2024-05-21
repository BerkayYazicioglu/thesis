%% Randim task allocation along a given path 
% assuming path{1} ~= robot.node

function [actions, fval, analysis] = random_task_allocator(robot, path)
    num_inputs = 10;

    % caching with a digraph
    cache = digraph(0, ...
                    table("0", 0, seconds(0), robot.energy, {""}, false, 0, 0, ...
                   'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'flag', 'children', 'depth'}));
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
                t = cache.Nodes.t(idx);
                e = cache.Nodes.e(idx);
                flag = cache.Nodes.flag(idx);
                % if flagged, the branch is pruned
                if flag
                    final_state = "0" + erase(num2str(x), ' ');
                    if ~findnode(cache, final_state)
                        cache = cache.addnode(table(final_state, u, t, e, {""}, flag, 0, numel(x), ...
                            'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'flag', 'children', 'depth'}));
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
                flag = cache.Nodes.flag(idx);
                tau = tau{1};

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
                            for k = 1:length(performed)
                                u = u - sublist.u(p_idx(k)) / seconds(t);
                            end
                            tau = [tau; performed(:)];
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
                cache = cache.addnode(table(next_state, u, t, e, {tau}, flag, 0, j, ...
                     'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'flag', 'children', 'depth'}));
                cache = cache.addedge(state, next_state, 1);
                cache.Nodes.children(idx) = cache.Nodes.children(idx) + 1;
                state = next_state;
                % if flagged, no need to continue
                if flag && j < length(x)
                    final_state = "0" + erase(num2str(x), ' ');
                    cache = cache.addnode(table(final_state, u, t, e, {""}, flag, 0, numel(x), ...
                        'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'flag', 'children', 'depth'}));
                    cache = cache.addedge(state, final_state, 1);
                    break
                end
            end
        end
    end

    % construct random strings
    inputs = randi(length(robot.policy.configs.action_list), ...
        num_inputs, ...
        length(path));
    analysis.utility = zeros(1, num_inputs);
    fval = 0;
    actions = robot.policy.configs.action_list(ones(size(path)));
    for i = 1:num_inputs
        u = fitness(inputs(i,:));
        if u < fval
            fval = u;
            actions = robot.policy.configs.action_list(inputs(i,:));
        end
        analysis.utility(i) = -u;
    end
    fval = -fval;

    analysis.utility = sort(analysis.utility, 'descend');
    % calculate cache fullness
    cache_length = sum(cellfun(@(y) numel(y)==length(path)+1, cache.Nodes.Name));
    ratio = cache_length / length(robot.policy.configs.action_list)^length(path);
    analysis.log = sprintf('%s |fval: %.3f |size: %d |ratio: %.3f', ...
        robot.id, ...
        fval, ...
        cache_length, ...
        ratio);

end