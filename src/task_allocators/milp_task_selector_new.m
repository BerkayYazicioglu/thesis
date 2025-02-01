function pp = milp_task_selector_new(robot, preprocessing)
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

% construct gurobi model
model.A = sparse([]);
model.obj = [];
model.rhs = [];
model.sense = '';
model.vtype = '';
model.modelsense = 'max';
model.varnames = {};

%% Decision Variables
% Si (set selection)
S = zeros(n, 1);
for i = 1:n
    var_name = sprintf('S_%d', i);
    model.vtype = [model.vtype 'B']; % Binary variable
    model.varnames{end+1} = var_name;
    S(i) = numel(model.varnames); % Store index
end

% Ej (element selection)
E = zeros(height(preprocessing.outcomes), 1);
for j = 1: height(preprocessing.outcomes)
    var_name = sprintf('E_%d', j);
    model.vtype = [model.vtype 'B']; % Binary variable
    model.varnames{end+1} = var_name;
    E(j) = numel(model.varnames); % Store index
end

%% Set Variable Bounds
num_vars = numel(model.varnames);
model.lb = zeros(num_vars, 1);
model.ub = ones(num_vars, 1);

%% Constructing model.A (Constraints)
A = sparse(m + 2 * height(preprocessing.outcomes), num_vars);
rhs = [];
sense = '';
row_idx = 1;
% Element coverage constraint
for j = 1:m
    flags = preprocessing.outcomes.nodes == elements.nodes(j) & ...
            preprocessing.outcomes.task_types == elements.task_types(j);
    % include 1 for all y_ij corresponding to the same ej
    A(row_idx, E(flags)) = 1;
    row_idx = row_idx + 1;
    rhs = [rhs; allowed_intersections];
    sense = [sense; '<'];
end

% Set selection implications
for j = 1:height(preprocessing.outcomes)
    ej = preprocessing.outcomes(j, :);
    % forward relationship
    % include -1 for all x_i corresponding to sets containing ej, 1 for yij
    flags = preprocessing.outcomes.nodes == ej.nodes & ...
            preprocessing.outcomes.task_types == ej.task_types;
    [~, flags] = ismember(preprocessing.outcomes(flags, {'task_idx','actions'}), ...
                          sets(:, {'task_idx','actions'}), 'rows');
    A(row_idx, flags) = -1;
    A(row_idx, n + j) = 1;
    row_idx = row_idx + 1;
    rhs = [rhs; 0];
    sense = [sense; '<'];

    % backward relationship
    % include 1 for the corresponding x_i, -1 for the y_ij index
    flags = find(sets.task_idx == ej.task_idx & sets.actions == ej.actions);
    A(row_idx, flags) = 1;
    A(row_idx, n + j) = -1;
    row_idx = row_idx + 1;
    rhs = [rhs; 0];
    sense = [sense; '<'];
end

% objective function
model.A = A;
model.rhs = rhs;
model.sense = sense;
model.obj = zeros(1, num_vars);
for i = 1:n
    % objective function
    set = sets(i, :);
    U = dictionary("map", 0, "search", 0);
    flags = preprocessing.outcomes.task_idx == set.task_idx & ...
            preprocessing.outcomes.actions == set.actions;
    U(preprocessing.tasks(set.task_idx).type) = sum(preprocessing.outcomes.values(flags));
    t = T(1, set.task_idx+1) + preprocessing.dt(set.task_idx);
    model.obj(S(i)) = mcdm(robot.mission.mcdm, ...
                            1-min(t, t_max)/t_max, U("map"), U("search")); 
end

%% Run gurobi
params.outputflag = 1; % Display Gurobi output
result = gurobi(model, params);

if ~strcmp(result.status, 'OPTIMAL')
    error('Optimization was not successful. Status: %s\n', result.status);
end

x = result.x;
selected_sets = sets(find(x(S)), :);

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
selected_sets.u = nonzeros(model.obj(find(x(S))));

end