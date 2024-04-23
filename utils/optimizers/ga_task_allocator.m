%% Optimal task allocation along a given path with genetic optimization
% assuming path{1} ~= robot.node

function [actions, fval] = ga_task_allocator(robot, path)
    
    disp("======================")
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
    candidates = candidates(robot.policy.tasks.items.isKey(candidates));
    tasks = robot.policy.tasks.items(candidates);
    action_list = {"none", "explore", "search"};

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

    % caching with a digraph
    cache = digraph(0, ...
                    table("0", 0, seconds(0), robot.energy, {1:length(tasks)}, ...
                   'VariableNames', {'Name', 'u', 't', 'e', 'tau'}));


    %% Fitness function
    % 1 -> no action
    % 2 -> explore
    % 3 -> search
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

                % movement
                t = t + seconds(time_loss(j));
                e = e - energy_loss(j);

                % actions
                action = action_list{x(j)};
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
                        idx = tau(k);
                        if action == tasks(idx).type
                            c = tasks(idx).capability(robot, path{j});
                            if c >= tasks(idx).kill
                                u = u + (c * tasks(idx).weight * 100) / seconds(t);
                                idx_to_remove = [idx_to_remove k];
                            end
                        end
                    end
                    tau(idx_to_remove) = [];
                end
                % cache the next state
                cache = cache.addnode(table(next_state, u, t, e, {tau}, ...
                                     'VariableNames', {'Name', 'u', 't', 'e', 'tau'}));
                cache = cache.addedge(state, next_state, 1);
                state = next_state;
            end
        end
        u = -u;
    end

    %% Optimize the path task allocation using the fitness function
    disp("======================")
    popsize = 10;
    co_fraction = 0.6;
    sel_ratio = 0.2;
    elite_ratio = 0.2;
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
                           'MutationFcn', 'mutationpower', ...
                           'CrossoverFcn', 'crossoverscattered', ...
                           'MaxStallGenerations', max_stall, ...
                           'Display', 'iter', ...
                           'PlotFcn', plots);
    [x, fval, flag, output] = ...
                ga(@fitness, ...
                   length(path), ... % numvars
                   [],[],[],[], ...  % A, b, Aeq, Beq 
                   ones(size(path)), ... % lb
                   3*ones(size(path)), ... % up
                   [], ... % nonlcon
                   1:length(path), ... % intcon
                   options);
    actions = action_list(x);
    fval = -fval;
    % calculate cache fullness
    cache_length = sum(cellfun(@(y) numel(y)==length(x)+1, cache.Nodes.Name));
    ratio = cache_length / length(action_list)^length(x);
    fprintf('Cache size: %d | ratio: %.3f\n', cache_length, ratio);
end


