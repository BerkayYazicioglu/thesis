function tmax = milp_tmax(T_trans)
% n = 5; % Number of candidates
% Example transition time matrix (symmetric)
% T_trans = [ 0  1  3  4  5;
%             1  0  7  1  2;
%             3  7  0  2  3;
%             4  1  2  0  1;
%             5  2  3  1  0];

%% Initialize Gurobi Model
n = length(T_trans);
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

% Convert to Sparse Matrix
model.A = sparse(A);
model.rhs = rhs;
model.sense = sense;
model.obj = zeros(1, num_vars);
model.obj(X) = T_trans; % Maximize total time
model.modelsense = 'max';

%% Solve the Model with Gurobi
params.outputflag = 0; % Display Gurobi output
result = gurobi(model, params);

% ==========================
% Display Results
% ==========================
if strcmp(result.status, 'OPTIMAL')
    tmax = result.objval;
else
    error('Optimization was not successful. Status: %s\n', result.status);
end
end