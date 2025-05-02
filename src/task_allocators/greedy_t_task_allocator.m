function output = greedy_t_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

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
    output.cache_idx = 0;
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

all_nodes =  [robot.node preprocessing.tasks(sets.task_idx).node];
D = distance_matrix(robot, all_nodes, 2);
dt = zeros(1, height(sets) + 1);
de = zeros(1, height(sets) + 1);
dt(2:end) = seconds(preprocessing.dt(sets.task_idx));
de(2:end) = preprocessing.de(sets.task_idx);
T = D./robot.speed + repmat(dt, height(sets)+1, 1);
E = D * robot.energy_per_m + repmat(de, height(sets)+1, 1);
T(find(eye(height(sets)+1))) = 0;
T_mcdm_vals = T(:, 2:end);
T_mcdm_vals = T_mcdm_vals(T_mcdm_vals > 0);
t_max = max(T(1, :));

%% fitness function
function u = fitness(x)
    actions_ = sets.actions(x);
    u_ = 0;
    action_eval = 0;
    t_ = zeros(1, length(x));
    t_(1) = seconds(robot.time);
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
        t_mcdm_min = min(T_mcdm_vals);
        t_mcdm_max = max(T_mcdm_vals);
        t_mcdm(n) = 1 - (T(prev_task_idx+1, x(n)+1) - t_mcdm_min)/(t_mcdm_max - t_mcdm_min);
        if t_mcdm_max == t_mcdm_min
            t_mcdm(n) = 1;
        end
        % check constraints
        flag = check_constraints(preprocessing.constraints{set.task_idx}, ...
                                 seconds(t_(n)), ...
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
                         action_eval];
    end
    u = -u_;
end
 
%% calculate fitness values
% for i = 1:height(sets)
%     fitness(i);
% end
fitness(1);

% check if the robot needs to return to the charger
if isempty(cache)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = t_max;
    output.pp_task_idx = [];
    output.action_eval = NaN;
    output.cache_idx = 0;
else
    % find the best cache index
    if any(ismissing(cache),'all')
        error("NaN in cache")
    end
    cache.nodes = [cellfun(@(x) [preprocessing.tasks(x).node], cache.tasks, 'UniformOutput', false)];
    [max_u, max_row] = max(cache.u);
    output.tasks = preprocessing.tasks(cache.tasks{max_row});
    output.actions = cache.actions{max_row};
    output.charge_flag = false;
    output.u = max_u;
    output.cache = cache;
    output.t_max = t_max;
    output.pp_task_idx = 1:length(preprocessing.tasks);
    output.action_eval = cache.action_eval(max_row);
    output.cache_idx = max_row;
end

end

