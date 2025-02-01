clearvars; clc;

n = 5; % Number of candidates

% Example transition time matrix (symmetric)
T_trans = [ 0  1  3  4  5;
            1  0  7  1  2;
            3  7  0  2  3;
            4  1  2  0  1;
            5  2  3  1  0];

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

% % U_i (Subtour Elimination Variables)
% U = zeros(n, 1);
% for i = 1:n
%     var_name = sprintf('U_%d', i);
%     model.vtype = [model.vtype 'C']; % Continuous variable
%     model.varnames{end+1} = var_name;
%     U(i) = numel(model.varnames);
% end

% Total execution time T
var_name = 'T_total';
model.vtype = [model.vtype 'C']; % Continuous variable
model.varnames{end+1} = var_name;
T_total = numel(model.varnames);

% =========================================================================
% Define P_i (Integer Rank Variable: Position of candidate i in sequence)
P = zeros(n, 1);
for i = 1:n
    var_name = sprintf('P_%d', i);
    model.vtype = [model.vtype 'I']; % Integer variable
    model.varnames{end+1} = var_name;
    P(i) = numel(model.varnames);
end
% ========================================================================


%% Set Variable Bounds
num_vars = numel(model.varnames);
model.lb = -inf(num_vars, 1);
model.ub = inf(num_vars, 1);

% Bounds for Binary Variables
model.lb(X(:)) = 0;
model.ub(X(:)) = 1;

% Bounds for Continuous Variables
model.lb(T(:)) = 0; % Execution times are non-negative
% model.lb(U(:)) = 1; % Subtour elimination variable must be >= 1
model.lb(T_total) = 0; % Total time must be non-negative

model.lb(T(1)) = 0;
model.ub(T(1)) = 0;

% =========================================================================
model.lb(P(:)) = 1;
model.ub(P(:)) = n;
% =========================================================================


%% Constructing model.A (Constraints)
A = [];
rhs = [];
sense = '';

% Each candidate has exactly one outgoing transition
for i = 2:n
    row = zeros(1, num_vars);
    row(X(i, :)) = 1;
    A = [A; row];
    rhs = [rhs; 1];
    sense = [sense; '<'];
end

% Each candidate has exactly one incoming transition
for j = 2:n
    row = zeros(1, num_vars);
    row(X(:, j)) = 1;
    A = [A; row];
    rhs = [rhs; 1];
    sense = [sense; '='];
end

% Candidate 1 must not have an incoming transition
row = zeros(1, num_vars);
row(X(:,1)) = 1;  % Sum over all transitions into candidate 1
A = [A; row];
rhs = [rhs; 0];
sense = [sense; '='];

% Candidate 1 must have exactly one outgoing transition
row = zeros(1, num_vars);
row(X(1,2:n)) = 1;  % Sum over all transitions from candidate 1
A = [A; row];
rhs = [rhs; 1];
sense = [sense; '='];

% % Subtour elimination constraints (MTZ)
% for i = 2:n
%     for k = 2:n
%         if i ~= k
%             row = zeros(1, num_vars);
%             row(U(i)) = 1;
%             row(U(k)) = -1;
%             row(X(i, k)) = n;
%             A = [A; row];
%             rhs = [rhs; n - 1];
%             sense = [sense; '<'];
%         end
%     end
% end

% Tj - Ti - MXij >= Tij - M
% M = 1000; % Large constant
% for j = 2:n
%     for i = 1:n
%         if i ~= j
%             row = zeros(1, num_vars);
%             row(T(j)) = 1; % Ensure T_j accumulates correctly
%             row(T(i)) = -1;  % Carry forward execution time from previous node
%             row(X(i, j)) = -M; % Only applies if transition X(i,j) is selected
%             A = [A; row];
%             rhs = [rhs; -M + T_trans(i, j)];
%             sense = [sense; '>'];
%         end
%     end
% end

for j = 2:n
    for i = 1:n
        if i ~= j
            model.genconind(end+1).binvar = X(i, j);  % Binary variable X(i,j)
            model.genconind(end).binval = 1;  % Activate only when X(i,j) = 1
            model.genconind(end).a = zeros(1, num_vars);
            model.genconind(end).a(T(j)) = 1;  
            model.genconind(end).a(T(i)) = -1;
            model.genconind(end).rhs = T_trans(i, j); % Transition time value
            model.genconind(end).sense = '='; % Enforce equality
        end
    end
end

% ==========================================================================
% Ensure correct order: If X(i,j) = 1, then P_j = P_i + 1
for i = 1:n
    for j = 1:n
        if i ~= j
            row = zeros(1, num_vars);
            row(P(j)) = 1;
            row(P(i)) = -1;
            row(X(i, j)) = -n; % Big-M ensures constraint only activates for X(i,j) = 1
            A = [A; row];
            rhs = [rhs; 1 - n];
            sense = [sense; '>'];
        end
    end
end
% ==========================================================================

% Objective Function: Maximize total execution time
row = zeros(1, num_vars);
row(T_total) = -1;
for i = 1:n
    for k = 1:n
        row(X(i, k)) = T_trans(i, k);
    end
end

A = [A; row];
rhs = [rhs; 0];
sense = [sense; '='];

% Convert to Sparse Matrix
model.A = sparse(A);
model.rhs = rhs;
model.sense = sense;
model.obj = zeros(1, num_vars); % Initialize all coefficients to zero
model.obj(T_total) = 1; % Maximize total transition time

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
    fprintf('Execution Times T:\n');
    for j = 1:n
        fprintf('rank_%d: T_%d: %.4f\n', result.x(P(j)), j, result.x(T(j)));
    end
else
    fprintf('Optimization was not successful. Status: %s\n', result.status);
end
