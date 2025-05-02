function output = milp_lns_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

cache = table({}, {}, [], {}, {}, {}, {}, {}, [], ...
    'VariableNames', {'tasks', 'actions', 'u', 'u_map', 'u_search', 't_mcdm', 't', 'e', 'action_eval'});

if isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(0);
    output.pp_task_idx = [];
    output.action_eval = NaN;
    output.cache_idx = 0;
    return
end

[pp, pp_task_idx] = milp_task_selector(robot, preprocessing);

if isempty(pp.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(0);
    output.pp_task_idx = [];
    output.cache_idx = 0;
    return;
end

% construct tsp formulation
sets = groupcounts(pp.outcomes, ["task_idx" "actions"]);
sets.priority = zeros(height(sets), 1);
sets.norm = zeros(height(sets), 1); 
sets.capability = zeros(height(sets), 1);
flags = false(height(sets), 1);
for i = 1:height(sets)
    task = pp.tasks(sets.task_idx(i));
    if task.type == "map"
        sets.priority(i) = max(0, ...
            numel(robot.mission.world.environment.neighbors(task.node)) - ...
            numel(robot.mission.map.neighbors(task.node)));
        sets.norm(i) = max(sets.GroupCount);
        sets.capability(i) = robot.mapper.capability;
        flags(i) = true;
    else
        sets.priority(i) = task.priority;
        sets.norm(i) = max(sets.GroupCount);
        sets.capability(i) = robot.detector.capability;
    end
end
if sum(flags)
    sets.priority(flags) = sets.priority(flags) / (max(sets.priority(flags) + 0.0001));
end
n = height(sets) + 1;

% calculate all distance and travel time pairs
all_nodes = [robot.node pp.tasks(sets.task_idx).node];
D = distance_matrix(robot, all_nodes, 2);
dt = zeros(1, height(sets) + 1);
de = zeros(1, height(sets) + 1);
dt(2:end) = seconds(pp.dt(sets.task_idx));
de(2:end) = pp.de(sets.task_idx);
T = D./robot.speed + repmat(dt, n, 1);
E = D * robot.energy_per_m + repmat(de, n, 1);
T(find(eye(n))) = 0;

% approximate the maximum time
t_max = sum(T(:)); 
% T_trans = min(1, T ./ t_max);
T_trans = T;
T_const = zeros(n, height(pp.constraints{1}));
E_const = zeros(n, height(pp.constraints{1}));

% calculate wij for each candidate i
keys = robot.mission.mcdm.key;
mcdm_d = dictionary(...
    't', robot.mission.mcdm.weight(keys == 't'), ...
    'm', robot.mission.mcdm.weight(keys == 'm'), ...
    's', robot.mission.mcdm.weight(keys == 's'), ...
    'mt', robot.mission.mcdm.weight(keys == 'mt'), ...
    'st', robot.mission.mcdm.weight(keys == 'st'));
w = zeros(n, 3);
a = zeros(n, 1);
u = zeros(n, 1);
a_types = string.empty;
% wi1 -> mcdm(t)
% wi2 -> mcdm(t max(m s))
% wi3 -> mcdm(max(m s))
for i = 2:n
    set = sets(i-1, :);
    flags = pp.outcomes.task_idx == set.task_idx & ...
            pp.outcomes.actions == set.actions;

    a(i) = evalfis(robot.task_eval, ...
                  [set.capability ...
                  sum(flags) / set.norm ...
                  1 - median(pp.outcomes.distances(flags)) / max(pp.outcomes.distances(flags)), ...
                  set.priority]);
    a_types(i) = pp.tasks(set.task_idx).type;
    wi2 = 0;
    wi3 = 0;
    if pp.tasks(set.task_idx).type == "map"
        wi2 = mcdm_d('mt');
        wi3 = mcdm_d('m');
        u(i) = mcdm(robot.mission.mcdm, ...
                      1, a(i), 0);
    end
    if pp.tasks(set.task_idx).type == "search"
        wi2 = mcdm_d('st');
        wi3 = mcdm_d('s');
        u(i) = mcdm(robot.mission.mcdm, ...
                    1, 0, a(i));
    end
    w(i, :) = [mcdm_d('t') wi2 wi3];
    
    % constraints
    const = pp.constraints{i-1};
    for k = 1:height(const)
        % T_const(i,k) = seconds(const.Time(k) - robot.time) / t_max; 
        T_const(i,k) = seconds(const.Time(k) - robot.time);
        E_const(i,k) = const.energy(k);
    end
end

%% employ a milp solver
milp_output = milp_lns(T_trans, ...
                       E, ...
                       T_const, ...
                       E_const, ...
                       robot.energy, ...
                       w, ...
                       a, ...
                       u, ...
                       t_max, ...
                       min(robot.policy.prediction_horizon, height(sets)));


%% compile results
T_mcdm = T_trans;
T_vals = T_mcdm(:, 2:end);
T_vals = T_vals(T_vals > 0);
T_mcdm = (T_mcdm - min(T_vals)) / (max(T_vals) - min(T_vals));
T_mcdm(isnan(T_mcdm)) = 0;
T_mcdm(isinf(T_mcdm)) = 0;
T_mcdm = 1 - T_mcdm;

for i = 1:height(milp_output.cache)
    row = milp_output.cache(i, :);
    x_sol = row.x{1}(2:end) - 1;
    % t_sol = robot.time + seconds(row.t{1}(2:1+len_sol) * t_max);
    t_sol = seconds(robot.time) + row.t{1}(2:end);
    e_sol = row.e{1}(2:end);
    u_sol = row.u{1}(2:end);
    tasks_sol = pp_task_idx(sets.task_idx(x_sol));
    actions_sol = sets.actions(x_sol);
    u_map_sol = zeros(size(u_sol));
    u_search_sol = zeros(size(u_sol));
    u_map_sol([preprocessing.tasks(tasks_sol).type] == "map") = ...
        u_sol([preprocessing.tasks(tasks_sol).type] == "map");
    u_search_sol([preprocessing.tasks(tasks_sol).type] == "search") = ...
        u_sol([preprocessing.tasks(tasks_sol).type] == "search");
    t_mcdm_sol = [];
    for j = 1:length(row.x{1})-1
        t_mcdm_sol(end+1) = T_mcdm(row.x{1}(j), row.x{1}(j+1));
    end
    
    if any(ismissing(actions_sol)) || ...
       any(ismissing(u_map_sol)) || ...
       any(ismissing(u_search_sol)) || ...
       any(ismissing(t_sol)) || ...
       any(ismissing(e_sol)) || ...
       any(ismissing(u_sol))
        error("NaN in cache entry")
    end

    cache = [cache; {{tasks_sol(:)'}, ...
                     {actions_sol(:)'}, ...
                     row.u_total, ...
                     {u_map_sol(:)'}, ...
                     {u_search_sol(:)'}, ...
                     {t_mcdm_sol(:)'}}, ...
                     {t_sol(:)'}, ...
                     {e_sol(:)'}, ...
                     row.u_total];
end
len_sol = sum(milp_output.u > 0);
if len_sol > 0
    cache.nodes = [cellfun(@(x) [preprocessing.tasks(x).node], cache.tasks, 'UniformOutput', false)];
    len_sol = min(len_sol, robot.policy.control_horizon);
    x_sol = milp_output.x(2:1+len_sol) - 1;
    output.tasks = pp.tasks(sets.task_idx(x_sol));
    output.actions = sets.actions(x_sol)';
    output.charge_flag = false;
    output.u = milp_output.u_total;
    output.cache = cache;
    output.t_max = t_max;
    output.pp_task_idx = pp_task_idx;
    output.action_eval = milp_output.u_total;
    output.cache_idx = milp_output.cache_idx;
else
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(0);
    output.pp_task_idx = pp_task_idx;
    output.action_eval = NaN;
    output.cache_idx = 0;
end

end
