function [model, params, variables] = cooperation_milp(T_mcdm_all, ...
                                                       T_all, ...
                                                       W_all, ...
                                                       A_all, ...
                                                       schedule_starts, ...
                                                       schedule_lengths, ...
                                                       conflicts)

% Build the milp model
n = sum(schedule_lengths);
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

for k = 1:length(T_mcdm_all)
    % X (schedule transition selection matrix)
    X_idx = zeros(size(T_mcdm_all{k}));
    for i = 1:size(T_mcdm_all{k},1)
        for j = 1:size(T_mcdm_all{k},2)
            var_name = sprintf('X%d_%d_%d', k, i, j);
            model.vtype = [model.vtype 'B']; % Binary variable
            model.varnames{end+1} = var_name;
            X_idx(i, j) = numel(model.varnames); % Store index
        end
    end
    X_all_idx{end+1} = X_idx;
end

for k = 1:length(T_mcdm_all)
    % V (schedule selection)
    V_idx = zeros(schedule_lengths(k), 1);
    for j = 1:schedule_lengths(k)
        var_name = sprintf('V%d_%d', k, j);
        model.vtype = [model.vtype 'B'];
        model.varnames{end+1} = var_name;
        V_idx(j) = numel(model.varnames);
    end
    V_all_idx{end+1} = V_idx;
end

for k = 1:length(T_mcdm_all)
    % delta 
    delta_idx = zeros(schedule_lengths(k), 1);
    for j = 1:schedule_lengths(k)
        var_name = sprintf('delta%d_%d', k, j);
        model.vtype = [model.vtype 'B'];
        model.varnames{end+1} = var_name;
        delta_idx(j) = numel(model.varnames);
    end
    delta_all_idx{end+1} = delta_idx;
end

for k = 1:length(T_mcdm_all)
    % U (schedule utility)
    U_idx = zeros(schedule_lengths(k), 1);
    for j = 1:schedule_lengths(k)
        var_name = sprintf('U%d_%d', k, j);
        model.vtype = [model.vtype 'C'];
        model.varnames{end+1} = var_name;
        U_idx(j) = numel(model.varnames);
    end
    U_all_idx{end+1} = U_idx;
end

for k = 1:length(T_mcdm_all)
    % W (selected utility)
    W_idx = zeros(schedule_lengths(k), 1);
    for j = 1:schedule_lengths(k)
        var_name = sprintf('W%d_%d', k, j);
        model.vtype = [model.vtype 'C'];
        model.varnames{end+1} = var_name;
        W_idx(j) = numel(model.varnames);
    end
    W_all_idx{end+1} = W_idx;
end

for k = 1:length(T_mcdm_all)
    % T (accumulated time)
    T_idx = zeros(schedule_lengths(k), 1);
    for j = 1:schedule_lengths(k)
        var_name = sprintf('T%d_%d', k, j);
        model.vtype = [model.vtype 'C'];
        model.varnames{end+1} = var_name;
        T_idx(j) = numel(model.varnames);
    end
    T_all_idx{end+1} = T_idx;
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
end

% delta 
for i = 1:length(delta_all_idx)
    model.lb(delta_all_idx{i}(:)) = 0;
    model.ub(delta_all_idx{i}(:)) = 1;
end

% U 
for i = 1:length(U_all_idx)
    model.lb(U_all_idx{i}(:)) = 0;
    model.ub(U_all_idx{i}(:)) = 100;
    %model.ub(U_all_idx{i}(1)) = 0;
end

% W 
for i = 1:length(W_all_idx)
    model.lb(W_all_idx{i}(:)) = 0;
    model.ub(W_all_idx{i}(:)) = 100;
    %model.ub(W_all_idx{i}(1)) = 0;
end

% T 
for i = 1:length(T_all_idx)
    model.lb(T_all_idx{i}(:)) = 0;
    model.ub(T_all_idx{i}(:)) = 2;
    model.ub(T_all_idx{i}(1)) = 0;
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
    T_mcdm = T_mcdm_all{ii};
    T_trans = T_all{ii};
    w = W_all{ii};
    a = A_all{ii};

    % if a set is used, candidate 1 of that set must be selected
    % Vi <= V1
    for j = 2:size(X,1)
        row = zeros(1, num_vars);
        row(V(1)) = -1;
        row(V(j)) = 1;
        A = [A; row];
        rhs = [rhs; 0];
        sense = [sense; '<'];
    end

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

    % start nodes dont have incoming transitions
    row = zeros(1, num_vars);
    row(X(:,1)) = 1;
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

    % sum_i Xij = Vj
    for j = 2:size(X,1)
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

    % sum X = sum V - V1
    row = zeros(1, num_vars);
    row(X(:)) = 1;
    row(V(2:end)) = -1;
    A = [A; row];
    rhs = [rhs; 0];
    sense = [sense; '='];
    
    % only forward paths are feasible (lower triangle of X)
    for i = 1:size(X,1)
        for j = 1:i          % j ≤ i  
            model.ub(X(i,j)) = 0;   % force Xij = 0
        end
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

    for j = 1:size(X,1)
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
    end
end

% Encode conflicting choices
for i = 1:height(conflicts)
    c1 = conflicts.("1")(i);
    c2 = conflicts.("2")(i);
    v1_idx = find(c1 >= schedule_starts, 1, 'last');
    v2_idx = find(c2 >= schedule_starts, 1, 'last');

    % V1(c1) + V2(c2) <= 1;
    row = zeros(1, num_vars);
    row(V_all_idx{v1_idx}(c1 - schedule_starts(v1_idx) + 1)) = 1;
    row(V_all_idx{v2_idx}(c2 - schedule_starts(v2_idx) + 1)) = 1;
    A = [A; row];
    rhs = [rhs; 1];
    sense = [sense; '<'];
end

% set non-conflicting schedules to one
non_conflict = setdiff(1:n, unique(conflicts{:,:}(:)));
if ~isempty(non_conflict)
    for i = 1:length(non_conflict)
        c = non_conflict(i);
        v_idx = find(c >= schedule_starts, 1, 'last');
        V = V_all_idx{v_idx};
        if ~ismember(c, schedule_starts)
            % V(nc) = 1 
            row = zeros(1, num_vars);
            row(V(c - schedule_starts(v_idx) + 1)) = 1;
            A = [A; row];
            rhs = [rhs; 1];
            sense = [sense; '='];
        end
    end
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