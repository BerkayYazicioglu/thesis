function [pp, task_idx] = milp_task_selector(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

% ============================== params ===================================
max_work_limit = 15;
max_overlap = 2; 
% =========================================================================

if isempty(preprocessing.tasks)
    pp = preprocessing;
    task_idx = 1:length(pp.tasks);
    return
end

% calculate all distance and travel time pairs
all_nodes = [robot.node preprocessing.tasks.node];
D = distance_matrix(robot, all_nodes, 1);
T = seconds(D./robot.speed) + [seconds(0) preprocessing.dt];

% process sets
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

% add task types to preprocessing
preprocessing.outcomes.task_types = arrayfun(@(x) preprocessing.tasks(x).type, preprocessing.outcomes.task_idx);
preprocessing.outcomes.t = seconds(T(preprocessing.outcomes.task_idx + 1))';
preprocessing.outcomes.t = normalize(preprocessing.outcomes.t, 1, "range", [0.01 1]);
preprocessing.outcomes.values = zeros(height(preprocessing.outcomes), 1);
for i = 1:height(sets)
    flags = preprocessing.outcomes.task_idx == sets.task_idx(i) & ...
            preprocessing.outcomes.actions == sets.actions(i);
    action_eval = evalfis(robot.task_eval, ...
        [sets.capability(i) ...
        sets.GroupCount(i) / sets.norm(i) ...
        1 - median(preprocessing.outcomes.distances(flags))...
          / max(preprocessing.outcomes.distances(flags)), ...
        sets.priority(i)]);
    preprocessing.outcomes.values(flags) = action_eval / sum(flags) * ones(sum(flags), 1);
end

% construct the task optimization with maximum coverage
elements = groupsummary(preprocessing.outcomes, ["nodes" "task_types"], ...
    ["max", "mean", "median"], ["values", "t"]);
n = height(sets);
m = height(elements);

% construct gurobi model
model.A = sparse([]);
model.obj = [];
model.rhs = [];
model.sense = '';
model.vtype = '';
model.modelsense = 'max';
model.varnames = {};
model.genconind = struct.empty;

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
A = sparse(2 * m, num_vars);
rhs = [];
sense = '';
row_idx = 1;

% % select at most prediction horizon number of sets
% A(row_idx, S(:)) = 1;
% row_idx = row_idx + 1;
% rhs = [rhs; robot.policy.prediction_horizon];
% sense = [sense; '<'];

% select at most one set per task
% for j = 1:n
%     flags = sets.task_idx == sets.task_idx(j);
%     A(row_idx, S(flags)) = 1;
%     row_idx = row_idx + 1;
%     rhs = [rhs; 1];
%     sense = [sense; '<'];
% end

% Sij -> E(Sij)
M = max(elements.GroupCount) + 20;
for j = 1:n
    flags = preprocessing.outcomes.task_idx == sets.task_idx(j) & ...
            preprocessing.outcomes.actions == sets.actions(j);
    flags = ismember(elements.nodes, preprocessing.outcomes.nodes(flags)) & ...
            ismember(elements.task_types, preprocessing.outcomes.task_types(flags));

    % Sj -> sum(E(flags)) = |E(Sij)| 
    model.genconind(end+1).binvar = S(j);  
    model.genconind(end).binval = 1;  
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(E(flags)) = 1;  
    model.genconind(end).rhs = sets.GroupCount(j); 
    model.genconind(end).sense = '='; 

    % % sum(E(flags)) >= |E(Sij)| - M(1 - Sj);
    % A(row_idx, E(flags)) = 1;
    % A(row_idx, S(j)) = -M;
    % row_idx = row_idx + 1;
    % rhs = [rhs; sets.GroupCount(j) - M];
    % sense = [sense; '>'];
    % 
    % % sum(E(flags)) <= |E(Sij)| + M(1 - Sj);
    % A(row_idx, E(flags)) = 1;
    % A(row_idx, S(j)) = M;
    % row_idx = row_idx + 1;
    % rhs = [rhs; sets.GroupCount(j) + M];
    % sense = [sense; '<'];
end
% ej -> {at most max overlap S | ej in S}
M = height(sets) + 2;
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

    % sum({S | ej in S}) <= max_overlap  + M(1 - Ej)
    A(row_idx, S(flags)) = 1;
    A(row_idx, E(j)) = M;
    row_idx = row_idx + 1;
    rhs = [rhs; max_overlap + M];
    sense = [sense; '<'];
end

% objective function
model.A = A;
model.rhs = rhs;
model.sense = sense;
model.obj = zeros(1, num_vars);
% for i = 1:n
%     % objective function
%     set = sets(i, :);
%     U = dictionary("map", 0, "search", 0);
%     flags = preprocessing.outcomes.task_idx == set.task_idx & ...
%             preprocessing.outcomes.actions == set.actions;
%     U(preprocessing.tasks(set.task_idx).type) = sum(preprocessing.outcomes.values(flags));
%     t = T(1, set.task_idx+1) + preprocessing.dt(set.task_idx);
%     model.obj(S(i)) = mcdm(robot.mission.mcdm, ...
%                             1-min(t, t_max)/t_max, U("map"), U("search")); 
% end
for i = 1:m
    % objective function
    model.obj(E(i)) = elements.mean_values(i) / elements.mean_t(i);
end

%% Run gurobi
params.outputflag = 0; % Display Gurobi output
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

end