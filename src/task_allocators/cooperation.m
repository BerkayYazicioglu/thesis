function output = cooperation(mission, schedules, conflicts)
%COOPERATION Resolve the flagged conflicts between corresponding robots
%
% schedules -> combined schedules of all robots
% conflicts -> table of indices that are conflicting each other on the
%               combined schedules
max_s = 10;

tmax = seconds(max(schedules.Time) - min(schedules.Time));
T_all = {};
T_mcdm_all = {};
W_all = {};
A_all = {};
schedule_lengths = [];
schedule_starts = [];
idx = 1;
for i = 1:length(unique(schedules.robot_idx))
    rows = schedules(schedules.robot_idx == i, :);
    if isempty(rows)
        continue
    end
    robot = mission.robots(i);
    all_nodes = rows.node';
    D = distance_matrix(robot, all_nodes, 2);
    T = D./robot.speed;
    dt = zeros(1, length(all_nodes));
    for ii = 2:length(all_nodes)
        if rows.type(ii) == "map"
            dt(ii) = seconds(robot.mapper.t_s);
        else
            dt(ii) = seconds(robot.detector.t_s);
        end
    end
    T = T + repmat(dt, length(all_nodes), 1);
    T(find(eye(length(all_nodes)))) = 0;
    schedule_lengths = [schedule_lengths length(all_nodes)];
    schedule_starts = [schedule_starts idx];
    idx = idx + length(all_nodes);
    
    T_mcdm = T;
    T_vals = T_mcdm(:, 2:end);
    T_vals = T_vals(T_vals > 0);
    T_mcdm = (T_mcdm - min(T_vals)) / (max(T_vals) - min(T_vals));
    T_mcdm(isnan(T_mcdm)) = 0;
    T_mcdm(isinf(T_mcdm)) = 0;
    T_mcdm = 1 - T_mcdm;

    T_all{end+1} = T ./ tmax;
    T_mcdm_all{end+1} = T_mcdm;

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
    a = zeros(n, 1);
    % wi1 -> mcdm(t)
    % wi2 -> mcdm(t max(m s))
    % wi3 -> mcdm(max(m s))
    for j = 2:n
        row = rows(j,:);
        a(j) = row.u_map + row.u_search;
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
    end
    W_all{end+1} = w;
    A_all{end+1} = a;
end

%% run milp

[model, params, variables] = cooperation_milp(T_mcdm_all, ...
                                              T_all, ...
                                              W_all, ...
                                              A_all, ...
                                              schedule_starts, ...
                                              schedule_lengths, ...
                                              conflicts);
params.TimeLimit = 15;
params.MIPFocus = 1; 
params.outputflag = 1;
result = gurobi(model, params);

for i = 1:length(schedule_starts)
    X = reshape(result.x(variables.X{i}), schedule_lengths(i), schedule_lengths(i));
    V = result.x(variables.V{i});
    W = result.x(variables.W{i});
    U = result.x(variables.U{i});
    T = result.x(variables.T{i});
    delta = result.x(variables.delta{i});
end

%% process results
robot_idx = unique(schedules.robot_idx, 'stable');
for i = 1:length(robot_idx)
    robot = mission.robots(robot_idx(i));
    robot.schedule(:,:) = [];
    selected = result.x(variables.V{i});
    selected = selected(2:end);
    new_schedule = schedules(schedule_starts(i) + find(selected), :);
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

