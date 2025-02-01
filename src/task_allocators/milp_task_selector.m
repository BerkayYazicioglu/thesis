function pp = milp_task_selector(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

% ============================== params ===================================
allowed_intersections = 1;
% =========================================================================

if isempty(preprocessing.tasks)
    pp = preprocessing;
    return
end

% add task types to preprocessing
preprocessing.outcomes.task_types = arrayfun(@(x) preprocessing.tasks(x).type, preprocessing.outcomes.task_idx);

% construct the task optimization with maximum coverage
elements = groupcounts(preprocessing.outcomes, ["nodes" "task_types"]);
sets = groupcounts(preprocessing.outcomes, ["task_idx" "actions"]);
n = height(sets);
m = height(elements);

% calculate all distance and travel time pairs
all_nodes = [robot.node preprocessing.tasks.node];
D = distance_matrix(robot, all_nodes, 1);
T = seconds(D./robot.speed);
t_max = max(T + [seconds(0) preprocessing.dt]); 

% objective function
f = sparse(1, n + height(preprocessing.outcomes)); 
% (relaxed) element coverage constraint
A1 = sparse(m, n + height(preprocessing.outcomes));
b1 = allowed_intersections * ones(m, 1);
% set selection implication (forward)
A2 = sparse(height(preprocessing.outcomes), n + height(preprocessing.outcomes));
b2 = zeros(height(preprocessing.outcomes), 1);
% set selection implication (backward)
A3 = sparse(height(preprocessing.outcomes), n + height(preprocessing.outcomes));
b3 = zeros(height(preprocessing.outcomes), 1);

for i = 1:n
    % objective function
    set = sets(i, :);
    U = dictionary("map", 0, "search", 0);
    flags = preprocessing.outcomes.task_idx == set.task_idx & ...
            preprocessing.outcomes.actions == set.actions;
    U(preprocessing.tasks(set.task_idx).type) = sum(preprocessing.outcomes.values(flags));
    t = T(1, set.task_idx+1) + preprocessing.dt(set.task_idx);
    f(i) = -mcdm(robot.mission.mcdm, ...
                 1-min(t, t_max)/t_max, U("map"), U("search")); 
end
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
    row = preprocessing.outcomes(j, :);
    % constraint 2
    % include -1 for all x_i corresponding to sets containing ej, 1 for yij
    flags = preprocessing.outcomes.nodes == row.nodes & ...
            preprocessing.outcomes.task_types == row.task_types;
    [~, flags] = ismember(preprocessing.outcomes(flags, {'task_idx','actions'}), ...
                          sets(:, {'task_idx','actions'}), 'rows');
    A2j = sparse(1, n + height(preprocessing.outcomes));
    A2j(flags) = -1;
    A2j(n + j) = 1;
    A2(j, :) = A2j;

    % constraint 3
    % include 1 for the corresponding x_i, -1 for the y_ij index
    A3j = sparse(1, n + height(preprocessing.outcomes));
    flags = find(sets.task_idx == row.task_idx & sets.actions == row.actions);
    A3j(flags) = 1;
    A3j(n + j) = -1;
    A3(j, :) = A3j;
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
                       'AbsoluteGapTolerance', 0);
[x, fval, ~, result] = intlinprog(f, 1:length(f), A, b, A3, b3, lb, ub, options);
x = round(x);
selected_sets = sets(find(x(1:n)), :);

selected_tasks = unique(selected_sets.task_idx);
pp.outcomes = preprocessing.outcomes(find(x(n + 1:end)), :);
pp.outcomes.task_idx = arrayfun(@(x) find(selected_tasks == x), pp.outcomes.task_idx);
pp.tasks = preprocessing.tasks(selected_tasks);
pp.dt = preprocessing.dt(selected_tasks);
pp.de = preprocessing.de(selected_tasks);
pp.constraints = preprocessing.constraints(selected_tasks);

fprintf('ej : %d | yij: %d | selected: %d | init tasks/action: %d | final task/action % d | n tasks: %d \n', ...
         height(elements), height(preprocessing.outcomes), height(pp.outcomes), height(sets), height(selected_sets), length(selected_tasks));

% debug
loc = robot.world.get_coordinates([preprocessing.tasks(selected_sets.task_idx).node]);
selected_sets.X = loc(:,1) * 5;
selected_sets.Y = loc(:,2) * 5;
selected_sets.u = -nonzeros(f(find(x(1:n))));

end