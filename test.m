clearvars; clc;

n = 5; % Number of candidates

% Example transition time matrix (symmetric)
T = [ 0  1  3  4  5;
      1  0  7  1  2;
      3  7  0  2  3;
      4  1  2  0  1;
      5  2  3  1  0];

% Example weights (w_jk) and threshold values (a_j)
w = [0.5, 0.3, 0.2;  % w_11, w_12, w_13 for candidate 1
     0.6, 0.2, 0.2;  % w_21, w_22, w_23 for candidate 2
     0.4, 0.4, 0.2;  % w_31, w_32, w_33 for candidate 3
     0.7, 0.2, 0.1;  % w_41, w_42, w_43 for candidate 4
     0.5, 0.3, 0.2]; % w_51, w_52, w_53 for candidate 5

a = [0.5; 0.5; 0.5; 0.5; 0.5]; % Threshold values for each candidate

% approximate the maximum time
t_max = milp_tmax(T); 
T_trans = min(1, T ./ t_max);




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

% % P_i: integer rank variables for the MTZ constraints
% P = zeros(n,1);
% for i = 1:n
%     var_name = sprintf('P_%d', i);
%     model.vtype = [model.vtype, 'I'];  % integer
%     model.varnames{end+1} = var_name;
%     P(i) = numel(model.varnames);
% end

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

model.lb(U(:)) = 0;  % U_j must be non-negative
model.ub(U(:)) = inf; % No upper bound on U_j
model.lb(delta(:)) = 0; % Binary variable lower bound
model.ub(delta(:)) = 1; % Binary variable upper bound

% model.lb(P(:)) = 1;   % rank from 1..n
% model.ub(P(:)) = n;
% model.lb(P(1)) = 1;
% model.ub(P(1)) = 1;

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

% % 8) MTZ Rank Constraints: p_j = p_i + 1 if X(i,j)=1
% M_rank = n;  % or (n-1)
% for i = 1:n
%     for j = 1:n
%         if i ~= j
%             % p_j - p_i >= 1 - M_rank*(1 - X(i,j))
%             row = zeros(1, num_vars);
%             row(P(j))= +1;
%             row(P(i))= -1;
%             row(X(i,j))= -M_rank;
%             A = [A; row];
%             rhs= [rhs; 1 - M_rank];
%             sense= [sense; '>'];
% 
%             % p_j - p_i <= 1 + M_rank*(1 - X(i,j))
%             row = zeros(1, num_vars);
%             row(P(j))= +1;
%             row(P(i))= -1;
%             row(X(i,j))= +M_rank;
%             A = [A; row];
%             rhs= [rhs; 1 + M_rank];
%             sense= [sense; '<'];
%         end
%     end
% end

% =========================================================================
% Tj - Ti = Tij
% for j = 1:n
%     for i = 1:n
%         model.genconind(end+1).binvar = X(i, j);  % Binary variable X(i,j)
%         model.genconind(end).binval = 1;  % Activate only when X(i,j) = 1
%         model.genconind(end).a = zeros(1, num_vars);
%         model.genconind(end).a(T(j)) = 1;  
%         model.genconind(end).a(T(i)) = -1;
%         model.genconind(end).rhs = T_trans(i, j); % Transition time value
%         model.genconind(end).sense = '='; % Enforce equality
%     end
% end

% Big-M replacement for: Tj - Ti = T_trans(i,j) if X(i,j)=1
M = 2;  % Since T ∈ [0,1] and T_trans(i,j) < 1
for i = 1:n
    for j = 1:n
        % Tj - Ti >= T_trans(i,j) - M * (1 - X(i,j))
        row = zeros(1, num_vars);
        row(T(j)) = 1;       
        row(T(i)) = -1;      
        row(X(i,j)) = -M; 
        A = [A; row];
        rhs = [rhs; T_trans(i,j) - M];
        sense = [sense; '>'];

        % Tj - Ti <= T_trans(i,j) + M * (1 - X(i,j))
        row = zeros(1, num_vars);
        row(T(j)) = 1;
        row(T(i)) = -1;
        row(X(i,j)) = M;
        A = [A; row];
        rhs = [rhs; T_trans(i,j) + M];
        sense = [sense; '<'];
    end
end
% =========================================================================

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

% M_delta = 20; % or pick a suitable value based on w, a, T ranges
% for j = 1:n
%     % For readability, define the constant parts:
%     % expr1 = w_{j1}*((1 - T_j) - a_j) + w_{j2}*a_j
%     %        = w_{j1}*(1 - a_j) - w_{j1}*T_j + w_{j2}*a_j
%     % We'll handle -w_{j1}*T_j in the row directly.
% 
%     %------------------------------
%     % CASE 1: (delta_j=1) => U_j = w_{j1}*(1 - a_j) - w_{j1}*T_j + w_{j2}*a_j
%     % => U_j >= w_{j1}*(1 - a_j) - w_{j1}*T_j + w_{j2}*a_j - M_delta*(1 - delta_j)
%     row = zeros(1, num_vars);
%     row(U(j)) = 1;                
%     row(T(j)) =  w(j,1);          
%     row(delta(j))= -M_delta;         
%     constPart = w(j,1)*(1 - a(j)) + w(j,2)*a(j) - M_delta;
%     A   = [A; row];
%     rhs = [rhs; constPart];
%     sense = [sense; '>'];
% 
%     % => U_j <= expr1 + M_delta*(1 - delta_j)
%     row = zeros(1, num_vars);
%     row(U(j))  =  1;
%     row(T(j))  =  w(j,1);
%     row(delta(j))=  M_delta;   % + M_delta*(1 - delta_j) => -M_delta*delta_j on LHS
%     constPart    = w(j,1)*(1 - a(j)) + w(j,2)*a(j) + M_delta;
%     A   = [A; row];
%     rhs = [rhs; constPart];
%     sense = [sense; '<'];
% 
%     %------------------------------
%     % CASE 2: (delta_j=0) => U_j = expr2
%     % expr2 = w_{j3}*(a_j - (1 - T_j)) + w_{j2}*(1 - T_j)
%     %       = w_{j3}*a_j - w_{j3}*(1 - T_j) + w_{j2}*(1 - T_j)
%     %       = w_{j3}*a_j - w_{j3} + w_{j3}*T_j + w_{j2} - w_{j2}*T_j
%     % Rearrange properly for row:
% 
%     % => U_j >= expr2 - M_delta*delta_j
%     %    U_j - (some function of T_j) - M_delta*delta_j >= (constant terms)
%     % Expand carefully to isolate T(j) with the correct sign.
% 
%     % For clarity, let's define an expanded version:
%     % expr2 = [w_{j3}*(a_j - 1) + w_{j2}] + [ (w_{j3} + -w_{j2}) * T_j ]
%     row = zeros(1, num_vars);
%     row(U(j))    = 1;
%     % Coefficient for T(j) = (w_{j3} + -w_{j2})
%     row(T(j))    = ( w(j,3) - w(j,2) );
%     row(delta(j))= -M_delta;   % - M_delta*delta_j => + M_delta*(1 - delta_j) on LHS
%     % Constant part = w_{j3}*a_j - w_{j3}*1 + w_{j2}*1 = w_{j3}*(a_j -1) + w_{j2}
%     constPart = w(j,3)*(a(j) - 1) + w(j,2);
%     A   = [A; row];
%     rhs = [rhs; constPart];
%     sense = [sense; '>'];
% 
%     % => U_j <= expr2 + M_delta*delta_j
%     row = zeros(1, num_vars);
%     row(U(j))    = 1;
%     row(T(j))    = (w(j,3) - w(j,2));
%     row(delta(j))=  M_delta;
%     A   = [A; row];
%     rhs = [rhs; constPart];
%     sense = [sense; '<'];
% end

% for j = 1:n
%     % lambdaj = 1: 1 - Tj >= aj
%     model.genconind(end+1).binvar = delta(j);  % Binary variable
%     model.genconind(end).binval = 1;  % Activate when delta_j = 1
%     model.genconind(end).a = zeros(1, num_vars);
%     model.genconind(end).a(T(j)) = -1;  
%     model.genconind(end).rhs = a(j) - 1;  % Right-hand side
%     model.genconind(end).sense = '>';  % Enforces T_j >= a_j when delta_j = 1
% 
%     % lambdaj = 0: aj >= 1 - Tj
%     model.genconind(end+1).binvar = delta(j);  % Binary variable
%     model.genconind(end).binval = 0;  % Activate when delta_j = 1
%     model.genconind(end).a = zeros(1, num_vars);
%     model.genconind(end).a(T(j)) = 1;  
%     model.genconind(end).rhs = 1 - a(j);  % Right-hand side
%     model.genconind(end).sense = '>'; % Enforces T_j < a_j when delta_j = 0
% end

M_delta = 1;  % or bigger if needed
% (Case 1) If delta_j=1 => T_j <= (1 - a_j)
% T_j <= (1 - a_j) + M_delta * (1 - delta_j)
for j = 1:n
    row = zeros(1, num_vars);
    row(T(j)) = 1;         
    row(delta(j)) = M_delta;  % => T_j - M_delta*(1-delta_j) <= (1 - a_j)
    A = [A; row];
    rhs = [rhs; (1 - a(j)) + M_delta];
    sense = [sense; '<'];
end
% (Case 2) If delta_j=0 => T_j >= (1 - a_j)
% T_j >= (1 - a_j) - M_delta * delta_j
for j = 1:n
    row = zeros(1, num_vars);
    row(T(j)) = 1;      
    row(delta(j)) = M_delta;  % => T_j + M_delta*delta_j >= (1 - a_j)
    A = [A; row];
    rhs = [rhs; (1 - a(j))];
    sense = [sense; '>'];
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
%params.MIPFocus = 1; % Focus on finding feasible solutions faster

result = gurobi(model, params);

disp("milp task allocator s: " + result.runtime);
% ==========================
% Display Results
% ==========================
if strcmp(result.status, 'OPTIMAL')
    fprintf('Optimized Total Execution Time: %.4f\n', result.objval);

    % Extract solution
    xsol = result.x;
    X_opt = reshape(xsol(X(:)), [n, n]);
    T_sol = xsol(T(:));
    U_sol = xsol(U(:));
    delta_sol = xsol(delta(:));
    % P_sol = xsol(P(:));

    % Display
    disp('Optimized Transition Matrix X:');
    disp(X_opt);

    fprintf('Utility Values (U_j):\n');
    for j = 1:n
        fprintf('U_%d: %.4f\n', j, result.x(U(j)));
    end

    fprintf('\nElapsed Times T_j:\n');
    for j = 1:n
        fprintf('T(%d) = %.4f\n', j, T_sol(j));
    end

    % fprintf('\nRank Variables P_i:\n');
    % for i = 1:n
    %     fprintf('P(%d) = %.1f\n', i, P_sol(i));
    % end

    fprintf('\nDelta_j:\n');
    for j=1:n
        fprintf('delta(%d) = %.1f\n', j, delta_sol(j));
    end
else
    fprintf('Optimization was not successful. Status: %s\n', result.status);
end
