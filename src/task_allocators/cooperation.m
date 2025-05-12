function output = cooperation(mission, schedules, conflicts)
%COOPERATION Resolve the flagged conflicts between corresponding robots
%
% schedules -> combined schedules of all robots
% conflicts -> table of indices that are conflicting each other on the
%               combined schedules
max_s = 5;
t_max_relaxation = 1.5;


tmax = t_max_relaxation * seconds(max(schedules.Time) - min(schedules.Time));
T_all = {};
E_all = {};
W_all = {};
A_all = {};
schedule_starts = [];
robot_idx = unique(schedules.robot_idx);
action_evals = {};
constraints_all = {};
e0 = [];

for i = 1:length(robot_idx)
    constraints = {};
    robot = mission.robots(robot_idx(i));
    robot.update_maps();
    all_nodes = schedules.node';
    D = distance_matrix(robot, all_nodes, 2);
    T = D./robot.speed;
    E = D * robot.energy_per_m;
    dt = zeros(1, length(all_nodes));
    de = zeros(1, length(all_nodes));
    for ii = 1:length(all_nodes)
        if schedules.type(ii) == "map"
            dt(ii) = seconds(robot.mapper.t_s);
            de(ii) = robot.mapper.d_energy;
        elseif schedules.type(ii) == "search"
            dt(ii) = seconds(robot.detector.t_s);
            de(ii) = robot.detector.d_energy;
        end
    end
    E = E + repmat(de, length(all_nodes), 1);
    T = T + repmat(dt, length(all_nodes), 1);
    T(find(eye(length(all_nodes)))) = 0;
    T_all{end+1} = T ./ tmax;
    E_all{end+1} = E;
    e0(end+1) = robot.energy; 

    schedule_starts = [schedule_starts find(schedules.robot_idx == robot_idx(i), 1)];
    
    % calculate wij for each candidate i
    n = length(all_nodes);
    keys = robot.mission.mcdm.key;
    mcdm_d = dictionary(...
        't', robot.mission.mcdm.weight(keys == 't'), ...
        'm', robot.mission.mcdm.weight(keys == 'm'), ...
        's', robot.mission.mcdm.weight(keys == 's'), ...
        'mt', robot.mission.mcdm.weight(keys == 'mt'), ...
        'st', robot.mission.mcdm.weight(keys == 'st'));
    w = zeros(n, 3);
    a = zeros(n, 4);
    % wi1 -> mcdm(t)
    % wi2 -> mcdm(t max(m s))
    % wi3 -> mcdm(max(m s))
    for j = 1:n
        row = schedules(j, :);
        if row.type == "none"
            continue;
        end
        task_flags = row.node == [mission.tasks.node] ...
                   & row.type == [mission.tasks.type];
        task = mission.tasks(find(task_flags));
        preds = task.predict(mission.robots(row.robot_idx));
        if row.type == "map"
            priority = max(0, ...
                    numel(robot.mission.world.environment.neighbors(task.node)) - ...
                    numel(robot.mission.map.neighbors(task.node))) / ...
                    numel(robot.mission.world.environment.neighbors(task.node));
            capability = robot.mapper.capability;
            distance = 1 - median(preds.distances) / robot.mapper.max_range;
            quantity = height(preds);
        else
            priority = task.priority;
            capability = robot.detector.capability;
            distance = 1 - median(preds.distances) / robot.detector.max_range;
            quantity = height(preds);
        end

        a(j, :) = [priority capability distance quantity];
        wi2 = 0;
        wi3 = 0;
        if row.type == "map"
            wi2 = mcdm_d('mt');
            wi3 = mcdm_d('m');
        end
        if row.type == "search"
            wi2 = mcdm_d('st');
            wi3 = mcdm_d('s');
        end
        w(j, :) = [mcdm_d('t') wi2 wi3];

        % charge constraints
        constraints{j} = preprocess_constraints(robot, task);
        n_constraints = height(constraints{j});
    end
    W_all{end+1} = w;
    action_evals{end+1} = a;
    constraints_all{end+1} = constraints;
end

% process charge constraints
T_const_all = {};
E_const_all = {};

for i = 1:length(constraints_all)
    T_const = zeros(n, 1);
    E_const = zeros(n, 1);
    constraints = constraints_all{i};
    for j = 1:n
        const = constraints{j};
        if isempty(const)
            continue;
        end
        T_const(j) = min(1, seconds(const.Time(end) - robot.time) / tmax);
        E_const(j) = const.energy(end);
    end
    T_const_all{end+1} = T_const;
    E_const_all{end+1} = E_const;
end

% recalculate action utilities
map_flags = schedules.type == "map";
if any(map_flags)
    max_meas = max(cellfun(@(x) max(x(map_flags, 4)), action_evals));
    for i = 1:length(action_evals)
        action_evals{i}(map_flags, 4) = action_evals{i}(map_flags, 4) / max_meas;
    end
end

map_flags = schedules.type == "search";
if any(map_flags)
    max_meas = max(cellfun(@(x) max(x(map_flags, 4)), action_evals));
    for i = 1:length(action_evals)
        action_evals{i}(map_flags, 4) = action_evals{i}(map_flags, 4) / max_meas;
    end
end

for i = 1:length(action_evals)
    robot = mission.robots(robot_idx(i));
    a = zeros(n, 1);
    for j = 1:n
        a(j) = evalfis(robot.task_eval, ...
            [action_evals{robot_idx(i)}(j, 2), ...
             action_evals{robot_idx(i)}(j, 4), ...
             action_evals{robot_idx(i)}(j, 3), ...
             action_evals{robot_idx(i)}(j, 1)]);
    end
    A_all{end+1} = a;
end

%% run milp

[model, params, variables] = cooperation_milp(T_all, ...
                                              W_all, ...
                                              A_all, ...
                                              E_all, ...
                                              T_const_all, ...
                                              E_const_all, ...
                                              schedule_starts, ...
                                              e0, ...
                                              n, ...
                                              conflicts);
params.TimeLimit = max_s;
params.MIPFocus = 1; 
params.outputflag = 1;
result = gurobi(model, params);

for i = 1:length(schedule_starts)
    X = reshape(result.x(variables.X{i}), n, n);
    V = result.x(variables.V{i});
    W = result.x(variables.W{i});
    U = result.x(variables.U{i});
    T = result.x(variables.T{i});
    delta = result.x(variables.delta{i});
end

%% process results
for i = 1:length(robot_idx)
    robot = mission.robots(robot_idx(i));
    robot.schedule(:,:) = [];
    % calculate paths
    X = reshape(result.x(variables.X{i}(:)), [variables.n variables.n]);
    idx = schedule_starts(i);
    path = schedule_starts(i);
    while true
        next = find(X(idx, :) == 1);
        if isempty(next)
            break
        end
        path(end+1) = next;
        idx = next;
    end
    new_schedule = schedules(path, :);
    new_schedule(1, :) = [];
    if isempty(new_schedule)
        % new schedule needs to be calculated 
        task_types = ["map" "search"];
        task_idx = [];
        other_schedules = schedules(schedules.robot_idx ~= robot_idx(i),:);
        other_schedules = other_schedules(other_schedules.action ~= "none", :);
        for ii = 1:length(task_types)
            other_nodes = other_schedules.node(other_schedules.type == task_types(ii));
            other_nodes = unique(other_nodes);
            excluded_nodes = string.empty;
            for iii = 1:length(other_nodes)
                excluded_nodes = [excluded_nodes;
                                  other_nodes(iii);
                                  mission.map.nearest(other_nodes(iii), ...
                                                      mission.coordination_radius, ...
                                                      "Method", "unweighted")];
            end
            excluded_nodes = unique(excluded_nodes);
            flags = ismember([mission.tasks.node], excluded_nodes) & ...
                    ismember([mission.tasks.type], task_types(ii)) & ...
                    arrayfun(@(x) ismember(robot.id, x.R_k), mission.tasks);
            task_idx = [task_idx find(flags)];
        end
        % flag the robot on the excluded tasks
        for ii = 1:length(task_idx)
            r_idx = mission.tasks(task_idx(ii)).R_k == robot.id;
            mission.tasks(task_idx(ii)).R_k(r_idx) = [];
        end
        % plan new path
        output.pp.(robot.id) = robot.path_planner();
        % if charge_flag is raised, construct the return path
        if output.pp.(robot.id).charge_flag
            if robot.state == "idle"
                robot.schedule = timetable(min(other_schedules.Time), ...
                        robot.node, "charge_done", 100, ...
                        'VariableNames', {'node', 'action', 'energy'});
            else
                % override return schedule
                charger_nodes = [mission.charger.node;
                                 mission.charger.schedule.node];
                charger_times = [mission.charger.schedule.Time;
                                 seconds(inf)];
                robot.schedule = generate_return_path(robot, ...
                                                      robot.node, ...
                                                      robot.time, ...
                                                      robot.energy, ...
                                                      charger_nodes, ...
                                                      charger_times, ...
                                                      true);
                robot.return_schedule(:,:) = [];
            end 
        else
            robot.generate_schedule([output.pp.(robot.id).tasks.node], ...
                                    output.pp.(robot.id).actions, ...
                                    robot.time);
        end
        % unflag the tasks
        for ii = 1:length(task_idx)
            mission.tasks(task_idx(ii)).R_k(end+1) = robot.id;
        end
    else
        new_actions = new_schedule.action(1);
        new_nodes = new_schedule.node(1);
        robot.generate_schedule(new_nodes, new_actions, robot.time);
    end
end


output.u = result.objval;
output.trials = result.pool;
end

