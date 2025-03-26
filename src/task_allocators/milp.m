function [model, params, variables] = milp(T_trans, E_trans, T_const, E_const, e0, w, a)
%% Initialize Gurobi Model
n = numel(a);
n_const = size(T_const, 2);
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

% E_j (Energy)
E = zeros(n, 1);
for j = 1:n
    var_name = sprintf('E_%d', j);
    model.vtype = [model.vtype 'C']; % Continuous variable
    model.varnames{end+1} = var_name;
    E(j) = numel(model.varnames);
end

% Utility Variables (U_j)
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

% P_i: integer rank variables for the MTZ constraints
P = zeros(n,1);
for i = 1:n
    var_name = sprintf('P_%d', i);
    model.vtype = [model.vtype, 'I'];  % integer
    model.varnames{end+1} = var_name;
    P(i) = numel(model.varnames);
end

% Zij: candidate i satisfies charging constraint j
Z = zeros(n, n_const);
for i = 1:n
    for j = 1:n_const
        var_name = sprintf('Z_%d_%d', i, j);
        model.vtype = [model.vtype, 'B']; 
        model.varnames{end+1} = var_name;
        Z(i, j) = numel(model.varnames);
    end
end

% ksi_i: indicator for charge constraints
ksi = zeros(n,1);
for i = 1:n
    var_name = sprintf('ksi_%d', i);
    model.vtype = [model.vtype, 'B'];  
    model.varnames{end+1} = var_name;
    ksi(i) = numel(model.varnames);
end

% W_i: utility with the charge constraints
W = zeros(n,1);
for i = 1:n
    var_name = sprintf('W_%d', i);
    model.vtype = [model.vtype, 'C'];  
    model.varnames{end+1} = var_name;
    W(i) = numel(model.varnames);
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
model.ub(T(:)) = 1;
model.lb(T(1)) = 0;
model.ub(T(1)) = 0;

model.lb(E(:)) = -1000;
model.ub(E(:)) = 100;
model.lb(E(1)) = e0;
model.ub(E(1)) = e0;

model.lb(U(:)) = 0;  % U_j must be non-negative
model.ub(U(:)) = inf; % No upper bound on U_j
model.ub(U(1)) = 0; 
model.lb(delta(:)) = 0; % Binary variable lower bound
model.ub(delta(:)) = 1; % Binary variable upper bound
model.lb(delta(1)) = 0; 

model.lb(P(:)) = 1;   % rank from 1..n
model.ub(P(:)) = n;
model.lb(P(1)) = 1;
model.ub(P(1)) = 1;

model.lb(Z(:)) = 0;
model.ub(Z(:)) = 1;

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

% 8) MTZ Rank Constraints: p_j = p_i + 1 if X(i,j)=1
M_rank = n;  % or (n-1)
for i = 1:n
    for j = 1:n
        if i ~= j
            % p_j - p_i >= 1 - M_rank*(1 - X(i,j))
            row = zeros(1, num_vars);
            row(P(j))= +1;
            row(P(i))= -1;
            row(X(i,j))= -M_rank;
            A = [A; row];
            rhs = [rhs; 1 - M_rank];
            sense = [sense; '>'];

            % p_j - p_i <= 1 + M_rank*(1 - X(i,j))
            row = zeros(1, num_vars);
            row(P(j))= +1;
            row(P(i))= -1;
            row(X(i,j))= +M_rank;
            A = [A; row];
            rhs = [rhs; 1 + M_rank];
            sense = [sense; '<'];
        end
    end
end

% Tj - Ti = Tij if X(ij) = 1
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

% Ei - Ej = Eij if X(ij) = 1
for j = 1:n
    for i = 1:n
        model.genconind(end+1).binvar = X(i, j);  % Binary variable X(i,j)
        model.genconind(end).binval = 1;  % Activate only when X(i,j) = 1
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(E(j)) = -1;  
        model.genconind(end).a(E(i)) = 1;
        model.genconind(end).rhs = E_trans(i, j); % Transition energy value
        model.genconind(end).sense = '='; % Enforce equality
    end
end

for j = 1:n
    % Case 1: If delta_j = 1, enforce U_j = w_j1 * ((1 - T_j) - a_j) + w_j2 * a_j
    % U_j + w_j1 * Tj = w_j1 - wj_1 * a_j + w_j2 * a_j
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 1;  % Activate when delta_j = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(U(j)) = 1;  
    model.genconind(end).a(T(j)) = w(j,1);
    model.genconind(end).rhs = w(j, 1) - w(j,1) * a(j) + w(j,2) * a(j);  % Right-hand side
    model.genconind(end).sense = '=';  % Enforce equation
end
for j = 1:n
    % Case 2: If delta_j = 0, enforce U_j = w_j3 * (a_j - (1 - T_j)) + w_j2 * (1 - T_j)
    % U_j - w_j3 * T_j + w_j2 * T_j = w_j3 * a_j - w_j3 + w_j2
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 0;  % Activate when delta_j = 0
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(U(j)) = 1;  
    model.genconind(end).a(T(j)) = -w(j,3) + w(j,2);
    model.genconind(end).rhs = w(j,3) * a(j) - w(j,3) + w(j,2);  % Right-hand side
    model.genconind(end).sense = '=';  % Enforce equation
end

for j = 1:n
    % lambdaj = 1: 1 - Tj >= aj
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 1;  % Activate when delta_j = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(T(j)) = -1;  
    model.genconind(end).rhs = a(j) - 1;  % Right-hand side
    model.genconind(end).sense = '>';  % Enforces T_j >= a_j when delta_j = 1

    % lambdaj = 0: aj >= 1 - Tj
    model.genconind(end+1).binvar = delta(j);  % Binary variable
    model.genconind(end).binval = 0;  % Activate when delta_j = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(T(j)) = 1;  
    model.genconind(end).rhs = 1 - a(j);  % Right-hand side
    model.genconind(end).sense = '>'; % Enforces T_j < a_j when delta_j = 0
end

MT = 2;
ME = 101;
for i = 1:n
    for j = 1:n_const
        % Ti - T_const(ij) <= MT (1 - Z(i,j))
        row = zeros(1, num_vars);
        row(T(i)) = 1;
        row(Z(i,j)) = MT;
        A = [A; row];
        rhs = [rhs; MT + T_const(i,j)];
        sense = [sense; '<'];

        % Ei - E_const(ij) >= -ME (1 - Z(i,j))
        row = zeros(1, num_vars);
        row(E(i)) = 1;
        row(Z(i,j)) = -ME;
        A = [A; row];
        rhs = [rhs; -ME + E_const(i,j)];
        sense = [sense; '>'];
    end
end

for i = 1:n
    % ksi_i = 1: sum_j Zij >= 1
    model.genconind(end+1).binvar = ksi(i);  % Binary variable
    model.genconind(end).binval = 1;  % Activate when ksi_i = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(Z(i,:)) = 1;  
    model.genconind(end).rhs = 1;  % Right-hand side
    model.genconind(end).sense = '>';  

    % ksi_i = 0: sum_j Zij == 0
    model.genconind(end+1).binvar = ksi(i);  % Binary variable
    model.genconind(end).binval = 0;  % Activate when ksi_i = 0
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(Z(i,:)) = 1;  
    model.genconind(end).rhs = 0;  % Right-hand side
    model.genconind(end).sense = '=';  

    % ksi_i = 1: Wi = Ui
    model.genconind(end+1).binvar = ksi(i);  % Binary variable
    model.genconind(end).binval = 1;  % Activate when ksi_i = 1
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(W(i)) = 1;  
    model.genconind(end).a(U(i)) = -1;
    model.genconind(end).rhs = 0;  % Right-hand side
    model.genconind(end).sense = '=';  

    % ksi_i = 0: Wi == 0
    model.genconind(end+1).binvar = ksi(i);  % Binary variable
    model.genconind(end).binval = 0;  % Activate when ksi_i = 0
    model.genconind(end).a = zeros(1, num_vars);
    model.genconind(end).a(W(i)) = 1;  
    model.genconind(end).rhs = 0;  % Right-hand side
    model.genconind(end).sense = '=';  
end

% Convert to Sparse Matrix
model.A = sparse(A);
model.rhs = rhs;
model.sense = sense;
model.obj = zeros(1, num_vars);
model.obj(W(:)) = 1; % Maximize total utility
model.modelsense = 'max';

% Variable struct
variables.X = X;
variables.T = T;
variables.E = E;
variables.U = U;
variables.delta = delta;
variables.P = P;
variables.Z = Z;
variables.ksi = ksi;
variables.W = W;
variables.n = n;

% params
params.outputflag = 1; % Display Gurobi output
params.PoolSolutions = 1000;
%params.MIPFocus = 1; % Focus on finding feasible solutions faster

end