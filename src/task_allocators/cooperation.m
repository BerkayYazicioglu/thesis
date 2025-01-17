function output = cooperation(mission, schedules, conflicts)
%COOPERATION Resolve the flagged conflicts between corresponding robots
%
% schedules -> combined schedules of all robots
% conflicts -> table of indices that are conflicting each other on the
%               combined schedules
max_iter = 500;

% find the cache entry of the latest task allocation per robot
robot_caches = table();
T = dictionary();
t_cache = duration.empty; 

for r = 1:length(mission.robots)
    if ~ismember(r, schedules.robot_idx)
        continue
    end
    robot = mission.robots(r);
    [~, max_cache] = max(robot.cache.u);
    cache_row = robot.cache(max_cache, :);
    nodes = [cache_row.nodes{:}]';
    actions = [cache_row.actions{:}]';
    robot_idx = repmat(r, length(actions), 1);
    u_map = [cache_row.u_map{:}]';
    u_search = [cache_row.u_search{:}]';
    robot_caches = [robot_caches;
        table(robot_idx(robot.control_step+1:end), ...
              nodes(robot.control_step+1:end), ...
              actions(robot.control_step+1:end), ...
              u_map(robot.control_step+1:end), ...
              u_search(robot.control_step+1:end))];
    D = distance_matrix(robot, [robot.node nodes(robot.control_step+1:end)'], 2);
    T{r} = seconds(D./robot.speed);
    t_cache = [t_cache max([cache_row.t{:}])];
end
robot_caches.Properties.VariableNames = {'robot_idx', 'node', 'action', 'u_map', 'u_search'};
t_max = max(t_cache) - min([mission.robots.time]);
x_values = table2cell(conflicts);
x_values = unique([x_values{:}]);
conflict_lookup = zeros(size(conflicts));
for i = 1:size(conflict_lookup,1)
    conflict_lookup(i,1) = find(x_values == conflicts.('1')(i));
    conflict_lookup(i,2) = find(x_values == conflicts.('2')(i));
end

%% fitness function for integer choices between conflicts
% x -> [x1 ... xn] denoting indices on conflict indices x_values
%    xi -> 0, 1 binary decision variable (0 -> delete | 1 -> keep)
function result = fitness(x)
    % calculate timings and utilities per robot cache with removed entries 
    removed_rows = schedules(x_values(x == 0), :);
    u = 0;
    robot_idx_ = unique(robot_caches.robot_idx);

    for k = 1:length(robot_idx_)
        robot_cache = robot_caches(robot_caches.robot_idx == robot_idx_(k),:);
        robot_removed = removed_rows(removed_rows.robot_idx == robot_idx_(k),:);
        prev = 1;
        t_cur = seconds(0);
        for kk = 1:height(robot_cache)
            flag_node = ismember(robot_removed.node, robot_cache.node(kk));
            flag_action = ismember(robot_removed.action, robot_cache.action(kk));
            % skip if row is included to remove
            if any(flag_node & flag_action)
                continue
            end
            dt = 0;
            type = extractBefore(robot_cache.action(kk), '_');
            if type == "map"
                dt = mission.robots(robot_idx_(k)).mapper.t_s;
            elseif type == "search"
                dt = mission.robots(robot_idx_(k)).detector.t_s;
            end
            t_cur = t_cur + T{robot_idx_(k)}(prev, kk+1) + dt;
            u = u + mcdm(mission.mcdm, ...
                         1-min(t_cur, t_max)/t_max, ...
                         robot_cache.u_map(kk), ...
                         robot_cache.u_search(kk));
            prev = kk+1;
        end
    end
    % check for constraints
    const = arrayfun(@(n) x(conflict_lookup(n,1)) + x(conflict_lookup(n,2)) - 1, ...
                     1:size(conflict_lookup,1));
    const(end+1) = 1 - sum(x);
    result.Fval = -u;
    result.Ineq = const;
end

%% distribute the best allocation of currently scheduled tasks
n = length(x_values);
srg_options = optimoptions('surrogateopt', ... 
                           'PlotFcn', {}, ...
                           'InitialPoints', ones(1,n), ...
                           'MaxFunctionEvaluations', max_iter, ...
                           'Display', 'none');
lb = zeros(1, n);
ub = ones(1, n);
[x, fval, ~, ~, trials] = surrogateopt(@fitness, lb, ub, 1:n, srg_options);

% configure robot schedules according to the best result
robot_idx = unique(schedules.robot_idx);
keep_schedules = schedules;
keep_schedules(x_values(x == 0),:) = [];

for i = 1:length(robot_idx)
    robot = mission.robots(robot_idx(i));
    new_schedule = keep_schedules(keep_schedules.robot_idx == robot_idx(i),:);
    robot.schedule(:,:) = [];
    if isempty(new_schedule)
        % if all tasks on the schedule are taken away, path plan again with
        % non-conflicting tasks
        task_types = ["map" "search"];
        task_idx = [];
        other_schedules = keep_schedules(keep_schedules.robot_idx ~= robot_idx(i),:);
        for ii = 1:length(task_types)
            other_nodes = other_schedules.node([other_schedules.type{:}] == task_types(ii));
            other_nodes = unique(other_nodes);
            excluded_nodes = string.empty;
            for iii = 1:length(other_nodes)
                excluded_nodes = [excluded_nodes;
                                  other_nodes(iii);
                                  mission.map.nearest(other_nodes(iii), ...
                                                      mission.coordination_radius, ...
                                                      "Method", "unweighted")];
            end
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
                robot.schedule = timetable(mission.time, ...
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
        new_actions = new_schedule.action;
        new_nodes = new_schedule.node;
        robot.generate_schedule(new_nodes, new_actions, robot.time);
    end
end

output.u = fval;
output.trials = trials;
end

