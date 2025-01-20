function output = ga_task_allocator(robot, preprocessing)
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

% calculate all distance and travel time pairs
all_nodes = [robot.node preprocessing.tasks.node];
D = distance_matrix(robot, all_nodes, 2);
T = seconds(D./robot.speed);

% approximate the maximum time
mean_dt = mean(preprocessing.dt);
t_max = seconds(0);
unvisited = 2:length(all_nodes); 
cur = 1;
for i = 1:prediction_horizon
    [~, sort_idx] = sort(T(cur, :), 'descend');
    sort_idx = sort_idx(ismember(sort_idx, unvisited));
    t_max = t_max + T(cur, sort_idx(1)) + mean_dt;
    unvisited(unvisited == sort_idx(1)) = [];
    cur = sort_idx(1);
end

cache = table({}, {}, [], {}, {}, {}, {}, ...
    'VariableNames', {'tasks', 'actions', 'u', 'u_map', 'u_search', 't', 'e'});
flagged = table({}, 'VariableNames', {'tasks'});

%% fitness function
function u = fitness(x)
    tasks_ = preprocessing.tasks(x);
    actions_ = string.empty;
    u_ = 0;
    t_ = seconds(zeros(1, length(x)));
    e_ = zeros(1, length(x));
    u_map = zeros(1, length(x));
    u_search = zeros(1, length(x)); 
    flag = true; 
    start_n = 1;

    % check if a portion of x is flagged infeasable
    [prev_x, ~] = lcss(flagged.tasks, x);
    if ~isempty(prev_x)
        prev_idx = find(cellfun(@(x_) x_ == prev_x, flagged.tasks), 1);
        if ~isempty(prev_idx)
            u = inf;
            return
        end
    end
    
    % check if a portion of x is already calculated
    [prev_x, cache_idx] = lcss(cache.tasks, x);
    if ~isempty(prev_x)
        prev_n = length(prev_x);
        actions_(1:prev_n) = cache.actions{cache_idx}(1:prev_n);
        t_(1:prev_n) = cache.t{cache_idx}(1:prev_n);
        e_(1:prev_n) = cache.e{cache_idx}(1:prev_n);
        u_map(1:prev_n) = cache.u_map{cache_idx}(1:prev_n);
        u_search(1:prev_n) = cache.u_search{cache_idx}(1:prev_n);
        % calculate the aggregated utility
        prev_u = mcdm(robot.mission.mcdm, ...
                      1-min(t_(1:prev_n), t_max)./t_max, ...
                      u_map(1:prev_n), ...
                      u_search(1:prev_n)); 
        u_ = sum(prev_u);
        start_n = prev_n + 1;
    end
    
    for n = start_n:length(x)
        % get nodes of current tasks with matching type
        cur_task_idx = x(n);
        prev_task_idx = 0;
        if n > 1
            prev_task_idx = x(n-1);
        end
        
        cur_type = preprocessing.tasks(cur_task_idx).type;
        cur_table = preprocessing.outcomes(preprocessing.outcomes.task_idx == cur_task_idx,:);
        if n > 1
            type_mask = [tasks_(1:n-1).type] == cur_type;
            if any(type_mask)
                prev_tasks = x(1:n-1);
                prev_tasks = prev_tasks(type_mask);
                prev_actions = actions_(type_mask);
                prev_nodes = [arrayfun(@(n_) preprocessing.outcomes.nodes( ...
                    preprocessing.outcomes.task_idx == prev_tasks(n_) & ...
                    preprocessing.outcomes.actions == prev_actions(n_))', ...
                    1:length(prev_tasks), 'UniformOutput', false)];
                % remove previous nodes from the current table
                cur_table(ismember(cur_table.nodes, [prev_nodes{:}]), :) = [];
            end
            t_cur = t_(n-1);
            e_cur = e_(n-1);
        else
            t_cur = seconds(0);
            e_cur = robot.energy;
        end
        cur_result = groupsummary(cur_table, 'actions', 'sum', 'values');
        t_cur = t_cur + T(prev_task_idx+1, cur_task_idx+1);
        e_cur = e_cur - D(prev_task_idx+1, cur_task_idx+1) * robot.energy_per_m;
        if isempty(cur_result)
            actions_(n) = "none";
            u_map(n) = 0;
            u_search(n) = 0;
            u_cur = u_;
        else
            U_ = dictionary("map", 0, "search", 0);
            [U_(cur_type), max_idx] = max(cur_result.sum_values);
            actions_(n) =  cur_result.actions(max_idx);
            t_cur = t_cur + preprocessing.dt(cur_task_idx);
            e_cur = e_cur - preprocessing.de(cur_task_idx);
            u_cur = u_ + mcdm(robot.mission.mcdm, ...
                              1-min(t_cur, t_max)/t_max, ...
                              U_("map"), ...
                              U_("search")); 
            u_map(n) = U_("map");
            u_search(n) = U_("search");
        end
        t_(n) = t_cur;
        e_(n) = e_cur;
        % check constraints
        flag = check_constraints(preprocessing.constraints{cur_task_idx}, ...
                                 t_cur + robot.time, ...
                                 e_cur);
        if ~flag
            flagged = [flagged; {x(1:n)}];
            n = n - 1;
            break
        end
        u_ = u_cur;
    end
    % cache state
    if flag || n > 0
        cache = [cache; {{x(1:n)}, ...
                         {actions_(1:n)}, ...
                         u_, ...
                         {u_map(1:n)},...
                         {u_search(1:n)}, ...
                         {t_(1:n)}, ...
                         {e_(1:n)}}];
    end
    u = -u_;
end

%% create random unique combinations
comb = nchoosek(1:length(all_nodes)-1, prediction_horizon);
if size(comb,1) > max_iter
    comb = comb(randsample(size(comb,1), max_iter), :);
end

% iterate through the combinations and order the nodes
for i = 1:size(comb,1)
    mask = false(1, prediction_horizon+1);
    mask(comb(i,:)) = true;
    cur = 1;
    X = zeros(1, prediction_horizon);
    for k = 2:prediction_horizon+1
        remaining = find(mask);
        [~, mi] = min(T(cur, remaining));
        mi = remaining(mi);
        cur = mi;
        mask(mi) = false;
        X(k-1) = cur;
    end
end

% get the best result
if isempty(cache) || isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = t_max;
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
end

end