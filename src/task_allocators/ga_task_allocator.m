function output = ga_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

% ======= params ========
max_iter = 150;
n_population = 20;
elite_count = 2;
n_mut = 2;

% =======================
cache = table({}, {}, {}, [], {}, {}, {}, {}, {}, [], ...
    'VariableNames', {'sets', 'tasks', 'actions', 'u', 'u_map', 'u_search', 't_mcdm', 't', 'e', 'action_eval'});
flagged = table({}, 'VariableNames', {'sets'});

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
prediction_horizon = min(robot.policy.prediction_horizon, height(sets));

% calculate all distance and travel time pairs
all_nodes = [robot.node preprocessing.tasks(sets.task_idx).node];
D = distance_matrix(robot, all_nodes, 2);
T = seconds(D./robot.speed);

% approximate the maximum time
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
T_mcdm_vals = T(:, 2:end);
T_mcdm_vals = T_mcdm_vals(T_mcdm_vals > 0);

%% mutation function
function children = mutation(parents, options, k, fitness, state, score, pop)
    children = pop(parents, :); 
    for ii = 1:size(children, 1)
        child = children(ii,:);
        % pick 2 random cut points
        if k == 1
            idx = 1;
        else
            idx = randperm(k, 2);
        end
        startPt = min(idx);
        endPt = max(idx);
        % invert the substring
        child(startPt:endPt) = fliplr(child(startPt:endPt));
        complement = setdiff(1:height(sets), child);
        if ~isempty(complement)
            % pick a random position in [1..k]
            pos = randperm(k, min(numel(complement), n_mut));
            newVal = complement(randperm(numel(complement), length(pos)));
            child(pos) = newVal;
        end
        % Store updated child
        children(ii,:) = child;
    end
end

%% crossover function
function children = crossover(parents, options, k, fitness, ~, pop)
    children = zeros(length(parents)/2, k);
    for ii = 1:2:length(parents)
        parent1 = pop(parents(ii), :);
        parent2 = pop(parents(ii+1), :);
        children(ceil(ii/2) ,:) = orderCrossoverOneChild(parent1, parent2);
    end
   
    function child = orderCrossoverOneChild(p1, p2)
        n = length(p1);
        child = zeros(1,n);

        if n == 1
            if rand > 0.5
                child = p1;
            else
                child = p2;
            end
            return
        end
    
        % Choose two crossover points
        cx1 = randi([1, n-1]);
        cx2 = randi([cx1+1, n]);
    
        % Step 1: Copy slice from p1 into child
        child(cx1:cx2) = p1(cx1:cx2);
    
        % Step 2: Fill remaining positions from p2 in order, skipping duplicates
        used = ismember(p2, child(cx1:cx2)); % which elements of p2 are used in the slice
        p2filtered = p2(~used);             % elements of p2 not used in the slice
        outPositions = [1:(cx1-1), (cx2+1):n]; % positions to fill
        child(outPositions) = p2filtered(1:length(outPositions));
    end
end

%% fitness function
function u = fitness(x)
    % check if a portion of x is flagged infeasable
    [prev_x, ~] = lcss(flagged.sets, x);
    if ~isempty(prev_x)
        % check if the portion corresponds to the whole flagged entry
        prev_idx = find(cellfun(@(x_) isequal(x_, prev_x), flagged.sets), 1);
        if ~isempty(prev_idx)
            [cache_x, cache_idx] = lcss(cache.sets, prev_x);
            if ~isempty(cache_x)
                prev_n = length(cache_x);
                %t_ = cache.t{cache_idx}(1:prev_n);
                u_map = cache.u_map{cache_idx}(1:prev_n);
                u_search = cache.u_search{cache_idx}(1:prev_n);
                t_mcdm = cache.t_mcdm{cache_idx}(1:prev_n);
                % calculate the aggregated utility
                u = -mcdm(robot.mission.mcdm, ...
                          t_mcdm, u_map, u_search);
                u = sum(u);
            else
                u = inf;
            end
            return
        end
    end
    
    actions_ = sets.actions(x);
    u_ = 0;
    action_eval_ = 0;
    t_ = zeros(1, length(x));
    t_(1) = seconds(robot.time);
    e_ = robot.energy * ones(1, length(x));
    u_map = zeros(1, length(x));
    u_search = zeros(1, length(x)); 
    t_mcdm = zeros(1, length(x));
    flag = true; 
    start_n = 1;
    included = false(height(preprocessing.outcomes), 1);
    
    % check if a portion of x is already calculated
    [prev_x, cache_idx] = lcss(cache.sets, x);
    if ~isempty(prev_x)
        % check if x already was in the cache
        if isequal(prev_x, x)
            u = -cache.u(cache_idx);
            return
        end
        prev_n = length(prev_x);
        t_(1:prev_n) = cache.t{cache_idx}(1:prev_n);
        e_(1:prev_n) = cache.e{cache_idx}(1:prev_n);
        u_map(1:prev_n) = cache.u_map{cache_idx}(1:prev_n);
        u_search(1:prev_n) = cache.u_search{cache_idx}(1:prev_n);
        t_mcdm(1:prev_n) = cache.t_mcdm{cache_idx}(1:prev_n);
        % calculate the aggregated utility
        prev_u = mcdm(robot.mission.mcdm, ...
                      t_mcdm(1:prev_n), ...
                      u_map(1:prev_n), ...
                      u_search(1:prev_n)); 
        u_ = sum(prev_u);
        action_eval_ = cache.action_eval(cache_idx);
        start_n = prev_n + 1;
        % calculate included elements
        included = ismember(preprocessing.outcomes(:, {'task_idx','actions'}), ...
                            sets(x(1:prev_n), {'task_idx','actions'}), 'rows');
    end
    
    for n = start_n:length(x)
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
            flagged = [flagged; {x(1:n)}];
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
        if any(ismissing(x)) || ...
           any(ismissing(actions_)) || ...
           any(ismissing(u_map)) || ...
           any(ismissing(u_search)) || ...
           any(ismissing(t_mcdm)) || ...
           any(ismissing(t_)) || ...
           any(ismissing(e_))
            error("NaN in cache entry")
        end

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

%% sample combinations
x = [];
if prediction_horizon ~= 0 
    population = zeros(n_population, prediction_horizon);
    % iterate through the combinations and order the nodes
    for i = 1:size(population,1)
        population(i, :) = randperm(height(sets), prediction_horizon);
        % mask = false(1, prediction_horizon+1);
        % mask(population(i,:)) = true;
        % cur = 1;
        % X = zeros(1, prediction_horizon);
        % for k = 2:prediction_horizon+1
        %     remaining = find(mask);
        %     [~, mi] = min(T(cur, remaining));
        %     mi = remaining(mi);
        %     cur = mi;
        %     mask(mi) = false;
        %     X(k-1) = cur;
        % end
        % population(i,:) = X;
    end
    
    % construct ga optimizer
    ga_options = optimoptions('ga', ... 
                               'PopulationType', 'custom', ...
                               'PlotFcn', {'gaplotbestf'}, ...
                               'PopulationSize', n_population, ...
                               'InitialPopulationMatrix', population, ...
                               'MaxGenerations', max_iter, ...
                               'EliteCount', elite_count, ...
                               'SelectionFcn', @selectiontournament, ...
                               'CrossoverFcn', @crossover, ...
                               'MutationFcn', @mutation, ...
                               'Display', 'none');
    lb = ones(1, prediction_horizon);
    ub = height(sets) * ones(1, prediction_horizon);
    [x, fval, ~, trials, pop, scores] = ga(@fitness, ...
                                            prediction_horizon, ...
                                            [], [], [], [], ...
                                            [], [], ... %lb, ub, ...
                                            [], ...
                                            [], ... %1:prediction_horizon, ...
                                            ga_options);
    fprintf("   ga generations: %d | funccount: %d\n", trials.generations, trials.funccount);
end


% get the best result
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
    if any(ismissing(cache),'all')
        error("NaN in cache")
    end
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
    output.pp_task_idx = sets.task_idx(x);
    output.action_eval = cache.action_eval(max_row);
    output.cache_idx = max_row;
end

end