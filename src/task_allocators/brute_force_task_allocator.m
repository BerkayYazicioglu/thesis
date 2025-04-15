function output = brute_force_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

% parameters
max_iter = 100;
prediction_horizon = min(robot.policy.prediction_horizon, length(preprocessing.tasks));

cache = table({}, {}, {}, [], {}, {}, {}, {}, {}, [], ...
    'VariableNames', {'sets', 'tasks', 'actions', 'u', 'u_map', 'u_search', 't_mcdm', 't', 'e', 'action_eval'});

if isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(1);
    output.pp_task_idx = [];
    output.action_eval = NaN;
    return;
end

sets = groupcounts(preprocessing.outcomes, ["task_idx" "actions"]);
sets.priority = zeros(height(sets), 1);
sets.norm = zeros(height(sets), 1); 
sets.capability = zeros(height(sets), 1);
flags = false(height(sets), 1);
for i = 1:height(sets)
    task = preprocessing.tasks(sets.task_idx(i));
    if task.type == "map"
        sets.priority(i) = max(0, ...
            numel(robot.mission.world.environment.neighbors(task.node)) - ...
            numel(robot.mission.map.neighbors(task.node)));
        sets.norm(i) = robot.mapper.FoV_area;
        sets.capability(i) = robot.mapper.capability;
        flags(i) = true;
    else
        sets.priority(i) = task.priority;
        sets.norm(i) = robot.detector.FoV_area;
        sets.capability(i) = robot.detector.capability;
    end
end
if sum(flags)
    sets.priority(flags) = sets.priority(flags) / (max(sets.priority(flags) + 0.0001));
end

all_nodes =  [robot.node preprocessing.tasks(sets.task_idx).node];
D = distance_matrix(robot, all_nodes, 2);
dt = zeros(1, height(sets) + 1);
de = zeros(1, height(sets) + 1);
dt(2:end) = seconds(preprocessing.dt(sets.task_idx));
de(2:end) = preprocessing.de(sets.task_idx);
T = D./robot.speed + repmat(dt, height(sets)+1, 1);
E = D * robot.energy_per_m + repmat(de, height(sets)+1, 1);
T(find(eye(height(sets)+1))) = 0;

t_max = max(T(1, :));

%% fitness function
function u = fitness(x)
    actions_ = sets.actions(x);
    u_ = 0;
    action_eval_ = 0;
    t_ = zeros(1, length(x));
    e_ = robot.energy * ones(1, length(x));
    u_map = zeros(1, length(x));
    u_search = zeros(1, length(x)); 
    t_mcdm = zeros(1, length(x));
    flag = true; 
    included = false(height(preprocessing.outcomes), 1);
    
     for n = 1:length(x)
        set = sets(x(n), :);
        prev_task_idx = 0;
        if n > 1
            prev_task_idx = x(n-1);
        end
        t_(n) = t_(max(1, n-1)) + T(prev_task_idx+1, x(n)+1);
        e_(n) = e_(max(1, n-1)) - E(prev_task_idx+1, x(n)+1);
        T_mcdm = T(prev_task_idx+1, :);
        t_mcdm_min = min(T_mcdm(T_mcdm > 0));
        t_mcdm_max = max(T_mcdm(T_mcdm > 0));
        t_mcdm(n) = 1 - (T(prev_task_idx+1, x(n)+1) - t_mcdm_min)/(t_mcdm_max - t_mcdm_min);
        % check constraints
        flag = check_constraints(preprocessing.constraints{set.task_idx}, ...
                                 t_(n) + robot.time, ...
                                 e_(n));
        if ~flag
            n = n - 1;
            break
        end
        % get unincluded nodes of current tasks with matching type
        additions = preprocessing.outcomes.task_idx == set.task_idx & ...
                    preprocessing.outcomes.actions == set.actions & ...
                    ~included;
        included = included | additions;
        if all(~additions)
            u_map(n) = 0;
            u_search(n) = 0;
        else
            U_ = dictionary("map", 0, "search", 0);
            action_eval = evalfis(robot.task_eval, ...
               [sets.capability(x(n)) ...
                sum(additions) / sets.norm(x(n)) ...
                1 - median(preprocessing.outcomes.distances(additions)) / max(preprocessing.outcomes.distances(additions)), ...
                sets.priority(x(n))]);
            U_(preprocessing.tasks(set.task_idx).type) = action_eval;
            u_ = u_ + mcdm(robot.mission.mcdm, ...
                           t_mcdm(n), ...
                           U_("map"), ...
                           U_("search")); 
            u_map(n) = U_("map");
            u_search(n) = U_("search");
            if n == 1
                action_eval_ = action_eval;
            end
        end
    end

    % cache state
    if (flag || n > 0) & ...
        isempty(find(cellfun(@(x_) isequal(x_, x(1:n)), cache.sets), 1))
        cache = [cache; {{x(1:n)}, ...
                         {sets.task_idx(x(1:n))}, ...
                         {actions_(1:n)}, ...
                         u_, ...
                         {u_map(1:n)},...
                         {u_search(1:n)}, ...
                         {t_mcdm(1:n)}, ...
                         {t_(1:n)}, ...
                         {e_(1:n)}}, ...
                         action_eval_];
    end
    u = -u_;
end
 
%% create random unique combinations
for i = 1:max_iter
    pool = 1:height(sets);
    candidate = zeros(1, prediction_horizon);
    for j = 1:prediction_horizon
        c_idx = randi(length(pool));
        candidate(j) = pool(c_idx);
        pool(c_idx) = [];
    end
    % order the candidates wrt time
    T_cand = T(:, [1 candidate]);
    T_cand = T_cand([1 candidate], :);
    mask = true(1, prediction_horizon+1);
    mask(1) = false;
    X = zeros(1, prediction_horizon);
    cur = 1;
    for k = 2:prediction_horizon+1
        remaining = find(mask);
        [~, mi] = min(T_cand(cur, remaining));
        mi = remaining(mi);
        cur = mi;
        mask(mi) = false;
        X(k-1) = cur-1;
    end
    % calculate fitness
    fitness(candidate(X));
end

% get the best result
if isempty(cache) || isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = t_max;
    output.pp_task_idx = [];
    output.action_eval = NaN;
else
    cache.nodes = [cellfun(@(x) [preprocessing.tasks(x).node], cache.tasks, 'UniformOutput', false)];
    cache.t(:) = cellfun(@(x) x + robot.time, cache.t(:), 'UniformOutput', false); 
    [max_u, max_row] = max(cache.u);
    max_x = cache.tasks{max_row};
    num_control = min(length(max_x), robot.policy.control_horizon);
    actions = cache.actions{max_row};

    output.tasks = preprocessing.tasks(max_x(1:num_control));
    output.actions = actions(1:num_control);
    output.charge_flag = false;
    output.u = max_u;
    output.cache = cache;
    output.t_max = t_max;
    output.pp_task_idx = 1:length(preprocessing.tasks);
    output.action_eval = cache.action_eval(max_row);
end

end