%% Preprocess candidate tasks for each action along the path
% assuming path{1} ~= robot.node

function [candidates, time_loss, energy_loss] = preprocess_path(robot, path)

    candidates = table(1, "none", "0", 0, ...
        'VariableNames', {'path_idx', 'action', 'task_node', 'u'});
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
        for ii =  1:length(actions)
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
end