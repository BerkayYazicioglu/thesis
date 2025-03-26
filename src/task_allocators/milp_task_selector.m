function [pp, task_idx] = milp_task_selector(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

% ============================== params ===================================
max_work_limit = 5;
% =========================================================================

if isempty(preprocessing.tasks)
    pp = preprocessing;
    task_idx = 1:length(pp.tasks);
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
% Sij (set selection)
S = zeros(n, 1);
for i = 1:n
    var_name = sprintf('S_%d', i);
    model.vtype = [model.vtype 'B']; % Binary variable
    model.varnames{end+1} = var_name;
    S(i) = numel(model.varnames); % Store index
end

% Ej (element selection)
E = zeros(height(elements), 1);
for j = 1: height(elements)
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
A = sparse(2 * m + 3 * n + 1, num_vars);
rhs = [];
sense = '';
row_idx = 1;

% select at most prediction horizon number of sets
A(row_idx, S(:)) = 1;
row_idx = row_idx + 1;
rhs = [rhs; robot.policy.prediction_horizon];
sense = [sense; '<'];
% select at most one set per task
for j = 1:n
    flags = sets.task_idx == sets.task_idx(j);
    A(row_idx, S(flags)) = 1;
    row_idx = row_idx + 1;
    rhs = [rhs; 1];
    sense = [sense; '<'];
end
% Sij -> E(Sij)
M = max(elements.GroupCount) + 1;
for j = 1:n
    flags = preprocessing.outcomes.task_idx == sets.task_idx(j) & ...
            preprocessing.outcomes.actions == sets.actions(j);
    flags = ismember(elements.nodes, preprocessing.outcomes.nodes(flags)) & ...
            ismember(elements.task_types, preprocessing.outcomes.task_types(flags));

    % sum(E(flags)) >= |E(Sij)| - M(1 - Sj);
    A(row_idx, E(flags)) = 1;
    A(row_idx, S(j)) = -M;
    row_idx = row_idx + 1;
    rhs = [rhs; sets.GroupCount(j) - M];
    sense = [sense; '>'];

    % sum(E(flags)) <= |E(Sij)| + M(1 - Sj);
    A(row_idx, E(flags)) = 1;
    A(row_idx, S(j)) = M;
    row_idx = row_idx + 1;
    rhs = [rhs; sets.GroupCount(j) + M];
    sense = [sense; '<'];
end
% ej -> {exactly one S | ej in S}
M = height(sets) + 1;
for j = 1:m
    flags = elements.nodes(j) == preprocessing.outcomes.nodes & ...
            elements.task_types(j) == preprocessing.outcomes.task_types;
    flags = ismember(sets.task_idx, preprocessing.outcomes.task_idx(flags)) & ...
            ismember(sets.actions, preprocessing.outcomes.actions(flags));
    % sum({S | ej in S}) >= 1  - M(1 - Ej)
    A(row_idx, S(flags)) = 1;
    A(row_idx, E(j)) = -M;
    row_idx = row_idx + 1;
    rhs = [rhs; 1 - M];
    sense = [sense; '>'];

    % sum({S | ej in S}) <= 1  + M(1 - Ej)
    A(row_idx, S(flags)) = 1;
    A(row_idx, E(j)) = M;
    row_idx = row_idx + 1;
    rhs = [rhs; 1 + M];
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
params.WorkLimit = max_work_limit;
params.MIPFocus = 1; 
result = gurobi(model, params);

% disp("milp task selector s: " + result.runtime);

x = result.x;
selected_sets = sets(find(x(S)), :);
flags = false(height(preprocessing.outcomes), 1);
for i = 1:height(selected_sets)
    flags = flags | ...
           (selected_sets.task_idx(i) == preprocessing.outcomes.task_idx & ...
            selected_sets.actions(i) == preprocessing.outcomes.actions);
end
selected_tasks = unique(selected_sets.task_idx);
task_idx = selected_tasks;

pp.outcomes = preprocessing.outcomes(flags, :);
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