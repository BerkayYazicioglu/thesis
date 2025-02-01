function output = milp_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

if isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = table({}, {}, [], {}, {}, {}, {}, 'VariableNames', {'tasks', 'actions', 'u', 'u_map', 'u_search', 't', 'e'});
    output.t_max = seconds(0);
    return
end

pp = milp_task_selector(robot, preprocessing);

% employ ga to solve the ordering problem
%output = ga_task_allocator(robot, pp);

cache = table({}, {}, {}, [], {}, {}, {}, {}, ...
    'VariableNames', {'sets', 'tasks', 'actions', 'u', 'u_map', 'u_search', 't', 'e'});

if isempty(pp.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(1);
    return;
end

% construct tsp formulation
sets = groupcounts(pp.outcomes, ["task_idx" "actions"]);
n = height(sets) + 1;

% calculate all distance and travel time pairs
all_nodes = [robot.node preprocessing.tasks(sets.task_idx).node];
D = distance_matrix(robot, all_nodes, 2);
dt = [0 seconds(preprocessing.dt(sets.task_idx))];
T = D./robot.speed + repmat(dt', 1, n);
T(find(eye(n))) = 0;

% approximate the maximum time
t_max = milp_tmax(T); 
T_trans = 1 - min(1, T ./ t_max);

% calculate wij for each candidate i
keys = robot.mission.mcdm.key;
mcdm = dictionary('t', robot.mission.mcdm.weight(keys == 't'), ...
    'm', robot.mission.mcdm.weight(keys == 'm'), ...
    's', robot.mission.mcdm.weight(keys == 's'), ...
    'mt', robot.mission.mcdm.weight(keys == 'mt'), ...
    'st', robot.mission.mcdm.weight(keys == 'st'));
w = zeros(n, 3);
a = zeros(n, 1);
% wi1 -> mcdm(t)
% wi2 -> mcdm(t max(m s))
% wi3 -> mcdm(max(m s))
for i = 2:n
    set = sets(i-1, :);
    flags = preprocessing.outcomes.task_idx == set.task_idx & ...
            preprocessing.outcomes.actions == set.actions;
    a(i) = sum(preprocessing.outcomes.values(flags));
    wi2 = 0;
    wi3 = 0;
    if pp.tasks(set.task_idx).type == "map"
        wi2 = mcdm('mt');
        wi3 = mcdm('m');
    end
    if pp.tasks(set.task_idx).type == "search"
        wi2 = mcdm('st');
        wi3 = mcdm('s');
    end
    w(i, :) = [mcdm('t') wi2 wi3];
end

%% Initialize Gurobi Model
model.A = sparse([]);
model.obj = [];
model.rhs = [];
model.sense = '';
model.vtype = '';
model.modelsense = 'max';
model.varnames = {};
model.genconind = struct.empty;

%% Decision Variables
% X_ij (Transition Matrix)
X = zeros(n, n);
for i = 1:n
    for j = 1:n
        var_name = sprintf('X_%d_%d', i, j);
        model.vtype = [model.vtype 'B']; % Binary variable
        model.varnames{end+1} = var_name;
        X(i, j) = numel(model.varnames); % Store index
    end
end

% T_j (Elapsed Time)
T = zeros(n, 1);
for j = 1:n
    var_name = sprintf('T_%d', j);
    model.vtype = [model.vtype 'C']; % Continuous variable
    model.varnames{end+1} = var_name;
    T(j) = numel(model.varnames);
end

% Define P_i (Integer Rank Variable: Position of candidate i in sequence)
P = zeros(n, 1);
for i = 1:n
    var_name = sprintf('P_%d', i);
    model.vtype = [model.vtype 'I']; % Integer variable
    model.varnames{end+1} = var_name;
    P(i) = numel(model.varnames);
end

% % Utility Variables (U_j)
U = zeros(n, 1);
for j = 1:n
    var_name = sprintf('U_%d', j);
    model.vtype = [model.vtype 'C']; % Continuous variable
    model.varnames{end+1} = var_name;
    U(j) = numel(model.varnames);
end

% Binary Switch Variables (delta_j)
delta = zeros(n, 1);
for j = 1:n
    var_name = sprintf('delta_%d', j);
    model.vtype = [model.vtype 'B']; % Binary variable
    model.varnames{end+1} = var_name;
    delta(j) = numel(model.varnames);
end

%% Set Variable Bounds
num_vars = numel(model.varnames);
model.lb = -inf(num_vars, 1);
model.ub = inf(num_vars, 1);

% Bounds for Binary Variables
model.lb(X(:)) = 0;
model.ub(X(:)) = 1;

% Bounds for Continuous Variables
model.lb(T(:)) = 0; % Execution times are non-negative
% model.ub(T(:)) = 20; % Execution times are non-negative

model.lb(T(1)) = 0;
model.ub(T(1)) = 0;

model.lb(P(:)) = 1;
model.ub(P(:)) = n;
model.lb(U(:)) = 0;  % U_j must be non-negative
model.ub(U(:)) = inf; % No upper bound on U_j
model.lb(delta(:)) = 0; % Binary variable lower bound
model.ub(delta(:)) = 1; % Binary variable upper bound


%% Constructing model.A (Constraints)
A = [];
rhs = [];
sense = '';

% Each candidate has at most one outgoing transition
for i = 1:n
    row = zeros(1, num_vars);
    row(X(i, :)) = 1;
    A = [A; row];
    rhs = [rhs; 1];
    sense = [sense; '<'];
end

% Each candidate has at most one incoming transition
for j = 1:n
    row = zeros(1, num_vars);
    row(X(:, j)) = 1;
    A = [A; row];
    rhs = [rhs; 1];
    sense = [sense; '<'];
end

% sum Xij = n - 1
row = zeros(1, num_vars);
row(X(:)) = 1;  % Sum over all transitions
A = [A; row];
rhs = [rhs; n - 1];
sense = [sense; '='];

% Candidate 1 must not have an incoming transition
row = zeros(1, num_vars);
row(X(:,1)) = 1;  % Sum over all transitions into candidate 1
A = [A; row];
rhs = [rhs; 0];
sense = [sense; '='];

% Candidate 1 must have exactly one outgoing transition
row = zeros(1, num_vars);
row(X(1,:)) = 1;  % Sum over all transitions from candidate 1
A = [A; row];
rhs = [rhs; 1];
sense = [sense; '='];

% no self transitions: Xij = 0 for i = j
row = zeros(1, num_vars);
for i = 1:n
    row(X(i,i)) = 1;  % Sum over all transitions into candidate 1
end
A = [A; row];
rhs = [rhs; 0];
sense = [sense; '='];

% Tj - Ti = Tij
for j = 1:n
    for i = 1:n
        model.genconind(end+1).binvar = X(i, j);  % Binary variable X(i,j)
        model.genconind(end).binval = 1;  % Activate only when X(i,j) = 1
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(T(j)) = 1;  
        model.genconind(end).a(T(i)) = -1;
        model.genconind(end).rhs = T_trans(i, j); % Transition time value
        model.genconind(end).sense = '='; % Enforce equality
    end
end

% Ensure correct order: If X(i,j) = 1, then P_j = P_i + 1
for i = 1:n
    for j = 1:n
        model.genconind(end+1).binvar = X(i, j);  % Binary variable
        model.genconind(end).binval = 1;  % Activate when delta_j = 1
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(P(j)) = 1;  
        model.genconind(end).a(P(i)) = -1;
        model.genconind(end).rhs = 1;  % Right-hand side
        model.genconind(end).sense = '=';  % Enforce equation
    end
end
row = zeros(1, num_vars);
row(P(1)) = 1;
A = [A; row];
rhs = [rhs; 1];
sense = [sense; '='];

for j = 1:n
    % Case 1: If delta_j = 1, enforce U_j = w_j1 * (T_j - a_j) + w_j2 * a_j
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 1;  % Activate when delta_j = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(U(j)) = 1;  
    model.genconind(end).a(T(j)) = -w(j,1);
    model.genconind(end).rhs = -w(j,1) * a(j) + w(j,2) * a(j);  % Right-hand side
    model.genconind(end).sense = '=';  % Enforce equation
end
for j = 1:n
    % Case 2: If delta_j = 0, enforce U_j = w_j3 * (a_j - T_j) + w_j2 * T_j
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 0;  % Activate when delta_j = 0
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(U(j)) = 1;  
    model.genconind(end).a(T(j)) = w(j,3) - w(j,2);
    model.genconind(end).rhs = w(j,3) * a(j);  % Right-hand side
    model.genconind(end).sense = '=';  % Enforce equation
end

for j = 1:n
    % lambdaj = 1: Tj >= aj
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 1;  % Activate when delta_j = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(T(j)) = 1;  
    model.genconind(end).rhs = a(j);  % Right-hand side
    model.genconind(end).sense = '>';  % Enforces T_j >= a_j when delta_j = 1

    % lambdaj = 0: aj >= Tj
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 0;  % Activate when delta_j = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(T(j)) = 1;  
    model.genconind(end).rhs = a(j);  % Right-hand side
    model.genconind(end).sense = '<'; % Enforces T_j < a_j when delta_j = 0
end

% Convert to Sparse Matrix
model.A = sparse(A);
model.rhs = rhs;
model.sense = sense;
model.obj = zeros(1, num_vars);
model.obj(U(:)) = 1; % Maximize total utility
model.modelsense = 'max';


%% Solve the Model with Gurobi
params.outputflag = 1; % Display Gurobi output
result = gurobi(model, params);

% ==========================
% Display Results
% ==========================
if strcmp(result.status, 'OPTIMAL')
    fprintf('Optimized Total Execution Time: %.4f\n', result.objval);
    X_opt = reshape(result.x(X(:)), [n, n]);
    fprintf('Optimized Transition Matrix X:\n');
    disp(X_opt);
    fprintf('Utility Values (U_j):\n');
    for j = 1:n
        fprintf('U_%d: %.4f\n', j, result.x(U(j)));
    end
    for j = 1:n
        fprintf('rank_%d: T_%d: %.4f\n', result.x(P(j)), j, result.x(T(j)));
    end
else
    fprintf('Optimization was not successful. Status: %s\n', result.status);
end
