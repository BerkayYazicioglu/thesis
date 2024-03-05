%% Optimal task allocation along a given path with genetic optimization
% assuming path{1} ~= robot.node

function [tasks, fval] = ga_task_allocator(robot, path)
    % preprocess the tasks that can be done along the way
    candidates = [];
    for i = 1:length(path)
        [nodes, ~] = robot.visible_sensor.get_all(str2double(path{i}));
        candidates = [candidates; nodes(:)];
    end
    candidates = unique(candidates);
    candidates = arrayfun(@num2str, ...
                          candidates, ...
                          'UniformOutput', ...
                          false);
    candidates = candidates(robot.tasks.items.isKey(candidates));
    tasks = robot.tasks.items(candidates);
    actions = {"none", "explore", "search"};

    % calculate the timing between path points if no action is taken
    time_loss = zeros(1, length(path));
    energy_loss = zeros(1, length(path));
    node = robot.node;
    for i = 1:length(time_loss)
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
    function u = fitness(x)
        t = seconds(0);
        e = robot.energy;
        tau = tasks;
        u = 0;

        for j = 1:length(x)
            % movement
            t = t + seconds(time_loss(j));
            e = e - energy_loss(j);
            p = path{j};

            % actions
            action = actions{x(j)};
            if action ~= "none" 
                if action == "explore"
                    t = t + robot.visible_sensor.t_s;
                    e = e - robot.visible_sensor.d_energy;
                elseif action == "search"
                    t = t + robot.remote_sensor.t_s;
                    e = e - robot.remote_sensor.d_energy;
                end
                % get capabilities and weights of applicable tasks
                idx_to_remove = [];
                for k = 1:length(tau)
                    if action == tau(k).type
                        c = tau(k).capability(robot, p);
                        u = u + (c * tau(k).weight * 100) / seconds(t);
                        if c >= tau(k).kill
                            idx_to_remove = [idx_to_remove k];
                        end
                    end
                end
                tau(idx_to_remove) = [];
            end
        end
    end
    %% Optimize the path task allocation using the fitness function
    [x, fval] = ga(@fitness,length(path),[],[],[],[], ...
                   ones(size(path)), ...
                   3*ones(size(path)), ...
                   [], 1:length(path));
    tasks = actions(x);
end


