%% Optimal task allocation along a given path with brute force optimization
% assuming path{1} ~= robot.node

function [actions, fval, analysis] = brute_force_task_allocator(robot, path)
    
    % caching with a digraph
    cache = digraph(0, ...
                    table("0", 0, seconds(0), robot.energy, {[""]}, false, ...
                   'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'flag'}));

    % preprocess the tasks that can be done along the way
    candidates = table(1, "none", "0", 0, ...
        'VariableNames', {'path_idx', 'action', 'task_node', 'u'});
    rf = rowfilter(["path_idx", "action"]);

    time_loss = zeros(1, length(path));
    energy_loss = zeros(1, length(path));
    node = robot.node;
    row = 1;

    for i = 1:length(path)
        vec = [robot.world.X(str2double(path{i})) - robot.world.X(str2double(node)) ...
               robot.world.Y(str2double(path{i})) - robot.world.Y(str2double(node))];
        vec_angle = mod(atan2(vec(2), vec(1)), 2*pi);
        heading = find(round(robot.directions, 1) == round(vec_angle, 1));
        % get task nodes
        [nodes_v, ~] = unique(robot.visible_sensor.get_all(heading, str2double(path{i})));
        [nodes_r, ~] = unique(robot.remote_sensor.get_all(heading, str2double(path{i})));

        nodes_all = {nodes_v, nodes_r};
        actions = {"explore", "search"};
        for ii =  1:length(nodes_all)
            nodes = arrayfun(@num2str, nodes_all{ii}, 'UniformOutput', false);
            nodes = nodes(robot.policy.tasks.items.isKey(nodes));
            for iii = 1:length(nodes)
                task = robot.policy.tasks.items(nodes(iii));
                cap = robot.policy.capability(robot, ...
                                              path{i}, ...
                                              heading, ...
                                              task.node, ...
                                              actions{ii});
                if cap >= task.kill
                    % task is estimated to be accomplishable
                    candidates(row, :) = {i, ...
                                          task.type, ...
                                          task.node, ...
                                          cap * task.utility(robot)};
                    row = row + 1;
                end
            end
        end
        % calculate the timing between path points if no action is taken
        edge = findedge(robot.world.environment, node, path{i});
        distance = robot.world.environment.Edges(edge,:).Weight;
        time_loss(i) = distance / robot.speed;
        energy_loss(i) = distance * robot.energy_per_m;
        node = path{i};
    end

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
                u = cache.Nodes.u(cache.findnode(state));
            else
                % the state needs to be calculated and cached
                idx = cache.findnode(state);
                u = cache.Nodes.u(idx);
                t = cache.Nodes.t(idx);
                e = cache.Nodes.e(idx);
                tau = cache.Nodes.tau(idx);
                tau = tau{1};
                flag = false;

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
                        sublist = candidates(rf.path_idx == idx & rf.action == action(a), :);
                        [performed, p_idx] = setdiff(sublist.task_node, tau);
                        if ~isempty(p_idx)
                            for k = 1:length(performed)
                                u = u - sublist.u(p_idx(k)) / seconds(t);
                            end
                            tau = [tau; performed(:)];
                        end
                    end
                    % check if there was any improvement
                    flag = u == u_prev;
                end
                % aggregrate the utility of closer points on the path
                % u = u + u_prev;
                % cache the next state
                cache = cache.addnode(table(next_state, u, t, e, {tau}, flag, ...
                                     'VariableNames', {'Name', 'u', 't', 'e', 'tau', 'flag'}));
                cache = cache.addedge(state, next_state, 1);
                state = next_state;
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
            if cache.Nodes.flag(findnode(cache, new_node))
                cache = cache.rmnode(new_node);
            else
                leaf_nodes = [leaf_nodes new_node];
            end
        end
        leaf_nodes(leaf_idx) = [];
        % Find the index of the shortest string
        [~, leaf_idx] = min(cellfun(@strlength, leaf_nodes));
        depth = strlength(leaf_nodes{leaf_idx});
        traverse = depth <= length(path);
    end

    % get final utilities
    idx = find(cellfun(@(y) strlength(y)==length(path)+1, cache.Nodes.Name));
    fval = 0;
    actions = robot.policy.configs.action_list(ones(1, length(path)));

    % analysis data
    analysis.utility = zeros(1, length(idx));
    analysis.log = "";

    for ii = 1:length(idx)
        x = cache.Nodes.Name(idx(ii));
        u = cache.Nodes.u(idx(ii));
        if u < fval
            fval = u;
            x = str2num(char(num2cell(char(x))));
            actions = robot.policy.configs.action_list(x(2:end));
        end
        analysis.utility(ii) = -u; 
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


