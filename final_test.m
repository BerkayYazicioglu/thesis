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

% results
% Best objective 1.681250000000e+00, best bound 1.681250000000e+00, gap 0.0000%
% milp task allocator s: 0.131
% Optimized Total Execution Time: 1.6812
% Optimized Transition Matrix X:
%      0     1     0     0     0
%      0     0     0     1     0
%      0     0     0     0     0
%      0     0     0     0     1
%      0     0     1     0     0
% 
% Utility Values (U_j):
% U_1: 0.4000
% U_2: 0.3625
% U_3: 0.2500
% U_4: 0.3625
% U_5: 0.3062
% 
% Elapsed Times T_j:
% T(1) = 0.0000
% T(2) = 0.0625
% T(3) = 0.3750
% T(4) = 0.1250
% T(5) = 0.1875
% 
% Rank Variables P_i:
% P(1) = 1.0
% P(2) = 2.0
% P(3) = 5.0
% P(4) = 3.0
% P(5) = 4.0
% 
% Delta_j:
% delta(1) = 1.0
% delta(2) = 1.0
% delta(3) = 1.0
% delta(4) = 1.0
% delta(5) = 1.0


n = 12;
T_trans = ...
[0	0.0840884914627035	0.0864478826597805	0.0864479159943358	0.0864479159943358	0.0874251690858882	0.0874251710760265	0.0874251911642919	0.0874251991643151	0.0897845547186184	0.0897845563654476	0.0897846553591262;
0	0	0.0864478826597805	0.0864479159943358	0.0864479159943358	0.0874251690858882	0.0874251710760265	0.0874251911642919	0.0874251991643151	0.0897845547186184	0.0897845563654476	0.0897846553591262;
0.00235939119707699	0.0864478826597805	0	0.0888073071914128	0.0888073071914128	0.0864478768945948	0.0864478767905462	0.0897845823613689	0.0897845903613921	0.0888072625273250	0.0888072620799672	0.0921440465562031;
0.00235942453163225	0.0864479159943358	0.0888073071914128	0	0.0840884914627035	0.0897845936175205	0.0897845956076588	0.0864478768440029	0.0864478770703034	0.0921439792502507	0.0921439808970798	0.0874252308274939;
0.00235942453163225	0.0864479159943358	0.0888073071914128	0.0840884914627035	0	0.0897845936175205	0.0897845956076588	0.0864478768440029	0.0864478770703034	0.0921439792502507	0.0921439808970798	0.0874252308274939;
0.00333667762318467	0.0874251690858882	0.0864478768945948	0.0897845936175205	0.0897845936175205	0	0.0888072622224375	0.0907618687874766	0.0907618767874998	0.0864478770954337	0.0911666475118585	0.0931213329823108;
0.00333667961332300	0.0874251710760265	0.0864478767905462	0.0897845956076588	0.0897845956076588	0.0888072622224375	0	0.0907618707776149	0.0907618787776381	0.0911666478551677	0.0864478767521246	0.0931213349724492;
0.00333669970158834	0.0874251911642919	0.0897845823613689	0.0864478768440029	0.0864478768440029	0.0907618687874766	0.0907618707776149	0	0.0888072624516027	0.0931212544202067	0.0931212560670359	0.0864479743601165;
0.00333670770161157	0.0874251991643151	0.0897845903613921	0.0864478770703034	0.0864478770703034	0.0907618767874998	0.0907618787776381	0.0888072624516027	0	0.0931212624202300	0.0931212640670592	0.0897846136009777;
0.00569606325591486	0.0897845547186184	0.0888072625273250	0.0921439792502507	0.0921439792502507	0.0864478770954337	0.0911666478551677	0.0931212544202067	0.0931212624202300	0	0.0935260331445887	0.0954807186150410;
0.00569606490274404	0.0897845563654476	0.0888072620799672	0.0921439808970798	0.0921439808970798	0.0911666475118585	0.0864478767521246	0.0931212560670359	0.0931212640670592	0.0935260331445887	0	0.0954807202618702;
0.00569616389642261	0.0897846553591262	0.0921440465562031	0.0874252308274939	0.0874252308274939	0.0931213329823108	0.0931213349724492	0.0864479743601165	0.0897846136009777	0.0954807186150410	0.0954807202618702	0];

w = [0     0     0;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4;
     1     6     4];

a = [    0;
    0.0485;
    0.0862;
    0.3119;
    0.0884;
    0.1620;
    0.0741;
    0.1835;
    0.0817;
    0.2338;
    0.1531;
    0.1174];

T_trans = round(T_trans, 4);

%% Initialize Gurobi Model
model.A = sparse([]);
model.obj = [];
model.rhs = [];
model.sense = '';
model.vtype = '';
model.modelsense = 'max';
model.varnames = {};
%model.genconind = struct.empty;

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

% P_i: integer rank variables for the MTZ constraints
P = zeros(n,1);
for i = 1:n
    var_name = sprintf('P_%d', i);
    model.vtype = [model.vtype, 'I'];  % integer
    model.varnames{end+1} = var_name;
    P(i) = numel(model.varnames);
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

model.lb(U(:)) = 0;  % U_j must be non-negative
model.ub(U(:)) = 20; 
model.lb(delta(:)) = 0; % Binary variable lower bound
model.ub(delta(:)) = 1; % Binary variable upper bound

model.lb(P(:)) = 1;   % rank from 1..n
model.ub(P(:)) = n;
model.lb(P(1)) = 1;
model.ub(P(1)) = 1;

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
            rhs= [rhs; 1 - M_rank];
            sense= [sense; '>'];

            % p_j - p_i <= 1 + M_rank*(1 - X(i,j))
            row = zeros(1, num_vars);
            row(P(j))= +1;
            row(P(i))= -1;
            row(X(i,j))= +M_rank;
            A = [A; row];
            rhs= [rhs; 1 + M_rank];
            sense= [sense; '<'];
        end
    end
end

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

M_delta = 2; % or pick a suitable value based on w, a, T ranges
for j = 1:n
    % Uj = wj1 (1 - Tj - aj) + wj2 aj if deltaj = 1
    % Uj = wj3 (aj - 1 + Tj) + wj2 (1 -Tj) if deltaj = 0
    %------------------------------
    % Uj >= wj1 (1 - Tj - aj) + wj2 aj - M_delta*(1 - delta_j)
    % Uj + wj1 Tj - M deltaj >= wj1 - wj1 aj1  + wj2 aj - M
    row = zeros(1, num_vars);
    row(U(j)) = 1;                
    row(T(j)) = w(j,1);          
    row(delta(j))= -M_delta;
    A = [A; row];
    rhs = [rhs; w(j,1)*(1 - a(j)) + w(j,2)*a(j) - M_delta];
    sense = [sense; '>'];

    % Uj <= wj1 (1 - Tj - aj) + wj2 aj + M_delta*(1 - delta_j)
    % Uj + wj1 Tj + M deltaj <= wj1 - wj1 aj1  + wj2 aj + M
    row = zeros(1, num_vars);
    row(U(j)) = 1;
    row(T(j)) = w(j,1);
    row(delta(j))= M_delta;  
    A = [A; row];
    rhs = [rhs; w(j,1)*(1 - a(j)) + w(j,2)*a(j) + M_delta];
    sense = [sense; '<'];
    %------------------------------
    % Uj >= wj3 (aj - 1 + Tj) + wj2 (1 -Tj) - M_delta delta_j
    % Uj + M deltaj - wj3 Tj + wj2 Tj >= wj3 aj - wj3  + wj2
    row = zeros(1, num_vars);
    row(U(j)) = 1;
    row(T(j)) = w(j,2) - w(j,3);
    row(delta(j)) = M_delta;
    A = [A; row];
    rhs = [rhs; w(j,3)*(a(j) - 1) + w(j,2)];
    sense = [sense; '>'];

    % Uj <= wj3 (aj - 1 + Tj) + wj2 (1 -Tj) + M_delta delta_j
    % Uj - wj3 Tj + wj2 Tj - M deltaj <= wj3 aj - wj3 + wj2
    row = zeros(1, num_vars);
    row(U(j)) = 1;
    row(T(j)) = w(j,2) - w(j,3);
    row(delta(j)) = -M_delta;
    A = [A; row];
    rhs = [rhs; w(j,3)*(a(j) - 1) + w(j,2)];
    sense = [sense; '<'];
end

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
params.heuristics = 0.2;
params.RINS = 2;
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
    P_sol = xsol(P(:));

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

    fprintf('\nRank Variables P_i:\n');
    for i = 1:n
        fprintf('P(%d) = %.1f\n', i, P_sol(i));
    end

    fprintf('\nDelta_j:\n');
    for j=1:n
        fprintf('delta(%d) = %.1f\n', j, delta_sol(j));
    end
else
    fprintf('Optimization was not successful. Status: %s\n', result.status);
end
