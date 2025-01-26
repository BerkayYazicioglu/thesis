function output = milp_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

% ======= params ========
max_iter = 250;

lambda = 1;

% =======================
% calculate all distance and travel time pairs
all_nodes = [robot.node preprocessing.tasks.node];
D = distance_matrix(robot, all_nodes, 2);
T = seconds(D./robot.speed);
t_max = max(T(1,:) + [seconds(0) preprocessing.dt]); 

% add task types to preprocessing
preprocessing.outcomes.task_types = arrayfun(@(x) preprocessing.tasks(x).type, preprocessing.outcomes.task_idx);

% construct the task optimization with maximum coverage
elements = groupcounts(preprocessing.outcomes, ["nodes" "task_types"]);
sets = groupcounts(preprocessing.outcomes, ["task_idx" "actions"]);
n = height(sets);
m = height(elements);

% objective function
f = [lambda * ones(1, n) ./ n, zeros(1, height(preprocessing.outcomes))]; 
% (relaxed) element coverage constraint
A1 = sparse(m, n + height(preprocessing.outcomes));
b1 = ones(m, 1);
% set selection implication
A2 = sparse(height(preprocessing.outcomes), n + height(preprocessing.outcomes));
b2 = zeros(height(preprocessing.outcomes), 1);
% budget constraints 


for j = 1:m
    % constraint 1
    % include 1 for all y_ij corresponding to the same ej
    flags = preprocessing.outcomes.nodes == elements.nodes(j) & ...
            preprocessing.outcomes.task_types == elements.task_types(j);
    A1j = sparse(1, n + height(preprocessing.outcomes)); 
    A1j(n+1:end) = flags;
    A1(j,:) = A1j;
end

for j = 1:height(preprocessing.outcomes)
    % objective function
    w = 0;
    row = preprocessing.outcomes(j, :);
    % w_map
    if preprocessing.tasks(row.task_idx).type == "map"
        w = w + row.values * robot.mission.mcdm.weight(robot.mission.mcdm.key == "m");
    % w_search
    elseif preprocessing.tasks(row.task_idx).type == "search"
        w = w + row.values * robot.mission.mcdm.weight(robot.mission.mcdm.key == "s");
    end
    % w_time
    w_t = T(1, row.task_idx+1) + preprocessing.dt(row.task_idx);
    w_t = 1 - min(w_t, t_max) / t_max;
    w = w + w_t * robot.mission.mcdm.weight(robot.mission.mcdm.key == "t");
    f(n + j) = -w;

    % constraint 2
    % include 1 for all x_i corresponding to sets containing ej, -1 for yij
    flags = preprocessing.outcomes.nodes == row.nodes & ...
            preprocessing.outcomes.task_types == row.task_types;
    [~, flags] = ismember(preprocessing.outcomes(flags, {'task_idx','actions'}), ...
                          sets(:, {'task_idx','actions'}), 'rows');
    A2j = sparse(1, n + height(preprocessing.outcomes));
    A2j(flags) = -1;
    A2j(n + j) = 1;
    A2(j, :) = A2j;
end

% combine constraints
A = [A1; A2];
b = [b1; b2];

% variable bounds
lb = zeros(length(f), 1);
ub = ones(length(f), 1);

% solve milp
options = optimoptions('intlinprog', ...
                       'Display', 'none', ...
                       'AbsoluteGapTolerance', 0, ...
                       'OutputFcn', {'savemilpsolutions'});
[x, fval, ~, result] = intlinprog(f, 1:length(f), A, b, [], [], lb, ub, options);
x = round(x);



prediction_horizon = min(robot.policy.prediction_horizon, length(preprocessing.tasks));














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