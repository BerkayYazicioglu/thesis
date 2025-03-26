function output = milp_lns_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

cache = table({}, {}, [], {}, {}, {}, {}, ...
    'VariableNames', {'tasks', 'actions', 'u', 'u_map', 'u_search', 't', 'e'});

if isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(0);
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
    return;
end

% construct tsp formulation
sets = groupcounts(pp.outcomes, ["task_idx" "actions"]);
n = height(sets) + 1;

% calculate all distance and travel time pairs
all_nodes = [robot.node pp.tasks(sets.task_idx).node];
D = distance_matrix(robot, all_nodes, 2);
dt = [0 seconds(pp.dt(sets.task_idx))];
de = [0 pp.de(sets.task_idx)];
T = D./robot.speed + repmat(dt, n, 1);
E = D * robot.energy_per_m + repmat(de, n, 1);
T(find(eye(n))) = 0;

% approximate the maximum time
t_max = milp_tmax(T); 
T_trans = min(1, T ./ t_max);
T_const = zeros(n, height(preprocessing.constraints{1}));
E_const = zeros(n, height(preprocessing.constraints{1}));

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
    a(i) = sum(pp.outcomes.values(flags));
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
    const = preprocessing.constraints{i-1};
    for k = 1:height(const)
        T_const(i,k) = seconds(const.Time(k) - robot.time) / t_max; 
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
                       u);


%% compile results
for i = 1:height(milp_output.cache)
    row = milp_output.cache(i, :);
    len_sol = sum(row.u{1} > 0);
    if len_sol > 0
        x_sol = row.x{1}(2:1+len_sol) - 1;
        t_sol = robot.time + seconds(row.t{1}(2:1+len_sol) * t_max);
        e_sol = row.e{1}(2:1+len_sol);
        u_sol = row.u{1}(2:1+len_sol);
        tasks_sol = pp_task_idx(sets.task_idx(x_sol));
        actions_sol = sets.actions(x_sol);
        u_map_sol = zeros(size(u_sol));
        u_search_sol = zeros(size(u_sol));
        u_map_sol([preprocessing.tasks(tasks_sol).type] == "map") = ...
            u_sol([preprocessing.tasks(tasks_sol).type] == "map");
        u_search_sol([preprocessing.tasks(tasks_sol).type] == "search") = ...
            u_sol([preprocessing.tasks(tasks_sol).type] == "search");
        cache = [cache; {{tasks_sol(:)'}, ...
                         {actions_sol(:)'}, ...
                         row.u_total, ...
                         {u_map_sol(:)'}, ...
                         {u_search_sol(:)'}, ...
                         {t_sol(:)'}, ...
                         {e_sol(:)'}}];
    end
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
else
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(0);
end

end
