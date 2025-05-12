function [model, params, variables] = cooperation_milp(T_all, ...
                                                       W_all, ...
                                                       A_all, ...
                                                       E_all, ...
                                                       T_const_all, ...
                                                       E_const_all, ...
                                                       schedule_starts, ...
                                                       e0, ...
                                                       n, ...
                                                       conflicts)

% Build the milp model
model.A = sparse([]);
model.obj = [];
model.rhs = [];
model.sense = '';
model.vtype = '';
model.modelsense = 'max';
model.varnames = {};
model.genconind = struct.empty;

%% Decision variables
X_all_idx = {};
V_all_idx = {};
delta_all_idx = {};
U_all_idx = {};
W_all_idx = {};
T_all_idx = {};
E_all_idx = {};

for k = 1:length(T_all)
    % X (schedule transition selection matrix)
    X_idx = zeros(n,n);
    for i = 1:n
        for j = 1:n
            var_name = sprintf('X%d_%d_%d', k, i, j);
            model.vtype = [model.vtype 'B']; % Binary variable
            model.varnames{end+1} = var_name;
            X_idx(i, j) = numel(model.varnames); % Store index
        end
    end
    X_all_idx{end+1} = X_idx;
end

for k = 1:length(T_all)
    % V (schedule selection)
    V_idx = zeros(n, 1);
    for j = 1:n
        var_name = sprintf('V%d_%d', k, j);
        model.vtype = [model.vtype 'B'];
        model.varnames{end+1} = var_name;
        V_idx(j) = numel(model.varnames);
    end
    V_all_idx{end+1} = V_idx;
end

for k = 1:length(T_all)
    % delta 
    delta_idx = zeros(n, 1);
    for j = 1:n
        var_name = sprintf('delta%d_%d', k, j);
        model.vtype = [model.vtype 'B'];
        model.varnames{end+1} = var_name;
        delta_idx(j) = numel(model.varnames);
    end
    delta_all_idx{end+1} = delta_idx;
end

for k = 1:length(T_all)
    % U (schedule utility)
    U_idx = zeros(n, 1);
    for j = 1:n
        var_name = sprintf('U%d_%d', k, j);
        model.vtype = [model.vtype 'C'];
        model.varnames{end+1} = var_name;
        U_idx(j) = numel(model.varnames);
    end
    U_all_idx{end+1} = U_idx;
end

for k = 1:length(T_all)
    % W (selected utility)
    W_idx = zeros(n, 1);
    for j = 1:n
        var_name = sprintf('W%d_%d', k, j);
        model.vtype = [model.vtype 'C'];
        model.varnames{end+1} = var_name;
        W_idx(j) = numel(model.varnames);
    end
    W_all_idx{end+1} = W_idx;
end

for k = 1:length(T_all)
    % T (accumulated time)
    T_idx = zeros(n, 1);
    for j = 1:n
        var_name = sprintf('T%d_%d', k, j);
        model.vtype = [model.vtype 'C'];
        model.varnames{end+1} = var_name;
        T_idx(j) = numel(model.varnames);
    end
    T_all_idx{end+1} = T_idx;
end

for k = 1:length(T_all)
    % E (energy)
    E_idx = zeros(n, 1);
    for j = 1:n
        var_name = sprintf('E%d_%d', k, j);
        model.vtype = [model.vtype 'C'];
        model.varnames{end+1} = var_name;
        E_idx(j) = numel(model.varnames);
    end
    E_all_idx{end+1} = E_idx;
end


%% Set Variable Bounds
num_vars = numel(model.varnames);
model.lb = -inf(num_vars, 1);
model.ub = inf(num_vars, 1);

% Bounds for Binary Variables
for i = 1:length(X_all_idx)
    model.lb(X_all_idx{i}(:)) = 0;
    model.ub(X_all_idx{i}(:)) = 1;
end

% V 
for i = 1:length(V_all_idx)
    model.lb(V_all_idx{i}(:)) = 0;
    model.ub(V_all_idx{i}(:)) = 1;
    model.lb(V_all_idx{i}(schedule_starts(i))) = 1;
end

% delta 
for i = 1:length(delta_all_idx)
    model.lb(delta_all_idx{i}(:)) = 0;
    model.ub(delta_all_idx{i}(:)) = 1;
end

% U 
for i = 1:length(U_all_idx)
    model.lb(U_all_idx{i}(:)) = 0;
    model.ub(U_all_idx{i}(:)) = 20;
    %model.ub(U_all_idx{i}(1)) = 0;
end

% W 
for i = 1:length(W_all_idx)
    model.lb(W_all_idx{i}(:)) = 0;
    model.ub(W_all_idx{i}(:)) = 20;
    %model.ub(W_all_idx{i}(1)) = 0;
end

% T 
for i = 1:length(T_all_idx)
    model.lb(T_all_idx{i}(:)) = 0;
    model.ub(T_all_idx{i}(:)) = 1;
    model.ub(T_all_idx{i}(schedule_starts(i))) = 0;
end

% E
for i = 1:length(E_all_idx)
    model.lb(E_all_idx{i}(:)) = 0;
    model.ub(E_all_idx{i}(:)) = e0(i);
    model.lb(E_all_idx{i}(schedule_starts(i))) = e0(i);
end

%% Constructing model.A (Constraints)
A = [];
rhs = [];
sense = '';

for ii = 1:length(X_all_idx)
    X = X_all_idx{ii};
    V = V_all_idx{ii};
    delta = delta_all_idx{ii};
    U = U_all_idx{ii};
    W = W_all_idx{ii};
    T = T_all_idx{ii};
    E = E_all_idx{ii};
    T_trans = T_all{ii};
    E_trans = E_all{ii};
    T_const = T_const_all{ii}; 
    E_const = E_const_all{ii};
    w = W_all{ii};
    a = A_all{ii};

    % sum(X(:)) = sum(V(:)) - 1
    row = zeros(1, num_vars);
    row(X(:)) = 1;
    row(V(:)) = -1;
    A = [A; row];
    rhs = [rhs; -1];
    sense = [sense; '='];

    % start nodes dont have incoming transitions
    row = zeros(1, num_vars);
    row(X(:,schedule_starts)) = 1;
    A = [A; row];
    rhs = [rhs; 0];
    sense = [sense; '='];
   
    % no self transitions
    row = zeros(1, num_vars);
    for i = 1:size(X,1)
        row(X(i,i)) = 1;  % Sum over all transitions into candidate 1
    end
    A = [A; row];
    rhs = [rhs; 0];
    sense = [sense; '='];

    % allow a transition if only both i and j are visited
    for i = 1:size(X,1)
        for j = 1:size(X,2)
            if i ~= j
                % Xij <= Vi
                row = zeros(1, num_vars);
                row(X(i,j)) = 1;
                row(V(i)) = -1;
                A = [A; row];
                rhs = [rhs; 0];
                sense = [sense; '<'];
    
                % Xij <= Vj
                row = zeros(1, num_vars);
                row(X(i,j))= 1;
                row(V(j)) = -1;
                A = [A; row];
                rhs = [rhs; 0];
                sense = [sense; '<'];
            end
        end
    end

    % sum_i Xij = Vj
    for j = 1:size(X,1)
        if ismember(j, schedule_starts)
            continue;
        end
        row = zeros(1, num_vars);
        row(X(:,j)) = 1;
        row(V(j)) = -1;
        A = [A; row];
        rhs = [rhs; 0];
        sense = [sense; '='];
    end
    
    % sum_j Xij <= Vi
    for i = 1:size(X,1)
        row = zeros(1, num_vars);
        row(X(i,:)) = 1;
        row(V(i)) = -1;
        A = [A; row];
        rhs = [rhs; 0];
        sense = [sense; '<'];
    end

    % Tj - Ti = Xij T_all_ij
    for j = 1:size(X,1)
        for i = 1:size(X,2)
            model.genconind(end+1).binvar = X(i, j);  % Binary variable X(i,j)
            model.genconind(end).binval = 1;  % Activate only when X(i,j) = 1
            model.genconind(end).a = zeros(1, num_vars);
            model.genconind(end).a(T(j)) = 1;  
            model.genconind(end).a(T(i)) = -1;
            model.genconind(end).rhs = T_trans(i, j); % Transition time value
            model.genconind(end).sense = '='; % Enforce equality
        end
    end

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

    for j = 1:size(U,1)
        % Case 1: If delta_j = 1, enforce U_j = w_j1 * (1 - Tj - a_j) + w_j2 * a_j
        % U_j + w_j1 * Tj = wj_1 - wj_1 * a_j + w_j2 * a_j
        model.genconind(end+1).binvar = delta(j);  % Binary variable
        model.genconind(end).binval = 1;  % Activate when delta_j = 1
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(U(j)) = 1;  
        model.genconind(end).a(T(j)) = w(j,1); 
        model.genconind(end).rhs = w(j,1) - w(j,1) * a(j) + w(j,2) * a(j); 
        model.genconind(end).sense = '=';  % Enforce equation
    
        % Case 2: If delta_j = 0, enforce U_j = w_j3 * (a_j - 1 + Tj) + w_j2 * (1 - Tj)
        % U_j + (-w_j3 + wj2) Tj = -w_j3 + w_j3 * a_j + w_j2
        model.genconind(end+1).binvar = delta(j);  % Binary variable
        model.genconind(end).binval = 0;  % Activate when delta_j = 0
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(U(j)) = 1;  
        model.genconind(end).a(T(j)) = -w(j,3) + w(j,2); 
        model.genconind(end).rhs = -w(j,3) + w(j,3) * a(j) + w(j,2);  % Right-hand side
        model.genconind(end).sense = '=';  % Enforce equation
    
        % deltaj = 1: 1 - Tj >= aj
        model.genconind(end+1).binvar = delta(j);  % Binary variable
        model.genconind(end).binval = 1;  % Activate when delta_j = 1
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(T(j)) = -1;
        model.genconind(end).rhs = a(j) -1;  % Right-hand side
        model.genconind(end).sense = '>';  % Enforces T_j >= a_j when delta_j = 1
    
        % deltaj = 0: aj >= 1 - Tj
        model.genconind(end+1).binvar = delta(j);  % Binary variable
        model.genconind(end).binval = 0;  % Activate when delta_j = 1
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(T(j)) = 1;
        model.genconind(end).rhs = 1 - a(j);  % Right-hand side
        model.genconind(end).sense = '>';  

        % Vi = 0: Wi == 0
        model.genconind(end+1).binvar = V(j);  % Binary variable
        model.genconind(end).binval = 0;  % Activate when Vi = 0
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(W(j)) = 1;  
        model.genconind(end).rhs = 0;  % Right-hand side
        model.genconind(end).sense = '=';  

        % Vi = 1: Wi == Ui
        model.genconind(end+1).binvar = V(j);  % Binary variable
        model.genconind(end).binval = 1;  
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(W(j)) = 1;  
        model.genconind(end).a(U(j)) = -1;  
        model.genconind(end).rhs = 0;  % Right-hand side
        model.genconind(end).sense = '=';  

        % Vi = 1: Ti < Tconst_i
        model.genconind(end+1).binvar = V(j);  % Binary variable
        model.genconind(end).binval = 1;  
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(T(j)) = 1;  
        model.genconind(end).rhs = T_const(j);  % Right-hand side
        model.genconind(end).sense = '<';  

        % Vi = 1: Ei > Econst_i
        model.genconind(end+1).binvar = V(j);  % Binary variable
        model.genconind(end).binval = 1;  
        model.genconind(end).a = zeros(1, num_vars);
        model.genconind(end).a(E(j)) = 1;  
        model.genconind(end).rhs = E_const(j);  % Right-hand side
        model.genconind(end).sense = '>';  
    end
end

% Encode conflicting choices
for i = 1:height(conflicts)
    c1 = conflicts.("1")(i);
    c2 = conflicts.("2")(i);
    
    row = zeros(1, num_vars);
    % sum V(c1) + sum V(c2) <= 1;
    for j = 1:length(T_all)
        V = V_all_idx{j};
        row(V(c1)) = 1;
        row(V(c2)) = 1;
    end
    A = [A; row];
    rhs = [rhs; 1];
    sense = [sense; '<'];
end

% Only one candidate can be visited at all times
% Vi1 + Vj1 + .. <= 1
for i = 1:n
    row = zeros(1, num_vars);
    for j = 1:length(T_all)
        V = V_all_idx{j};
        row(V(i)) = 1;
    end
     A = [A; row];
    rhs = [rhs; 1];
    sense = [sense; '<'];
end



%% Convert to Sparse Matrix
model.A = sparse(A);
model.rhs = rhs;
model.sense = sense;
model.obj = zeros(1, num_vars);
for i = 1:length(W_all_idx)
    model.obj(W_all_idx{i}(:)) = 1; % Maximize total utility
end
model.modelsense = 'max';

% Variable struct
variables.X = X_all_idx;
variables.U = U_all_idx;
variables.delta = delta_all_idx;
variables.W = W_all_idx;
variables.V = V_all_idx;
variables.T = T_all_idx;
variables.n = n;

% params
params.outputflag = 1; % Display Gurobi output
params.PoolSolutions = 1000;
params.NumericFocus = 1; 
params.MIPFocus = 1;


% params.TimeLimit = 15;
% params.MIPFocus = 1; 
% params.outputflag = 1;
% result = gurobi(model, params);
end