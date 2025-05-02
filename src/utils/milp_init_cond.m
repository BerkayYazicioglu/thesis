function init_x = milp_init_cond(T_trans, E_trans, T_const, E_const, e0, w, a, pred_horizon, u)
% Calculate init x for the milp solver
n = numel(a);
n_const = size(T_const, 2);

X = zeros(n, n);
T = zeros(n, 1);
E = zeros(n, 1);
U = zeros(n, 1); 
delta = zeros(n, 1);
Z = zeros(n, n_const);
ksi = zeros(n,1);
W = zeros(n,1);
V = zeros(n,1);

T(1) = 0;
E(1) = e0;
U(1) = 0;
W(1) = 0;
delta(1) = 0;
V(1) = 1;

% greedy -> highest 'u' value to lowest
[~, idx] = sort(u(2:end), 'descend');
idx = idx(1:pred_horizon) + 1;
idx = [1; idx(:)];

V(idx) = 1;
for i = 1:length(idx)-1
    X(idx(i), idx(i+1)) = 1;
    T(idx(i+1)) = T(idx(i)) + T_trans(idx(i), idx(i+1));
    E(idx(i+1)) = E(idx(i)) - E_trans(idx(i), idx(i+1));
end

for j = 2:n
    tj = sum(X(:,j) .* T_trans(:,j) ./ max(T_trans(:,j)));
    % delta_j=1 => aj <= (1 - Tj)
    if 1 - tj >= a(j)
        delta(j) = 1;
    end
    % Uj = wj1 (1 - Tj - aj) + wj2 aj if deltaj = 1
    % Uj = wj3 (aj - 1 + Tj) + wj2 (1 -Tj) if deltaj = 0
    if delta(j) == 1
        U(j) = w(j,1) * (1 - tj - a(j)) + w(j,2) * a(j);
    else
        U(j) = w(j,3) * (a(j) - 1 + tj) + w(j,2) * (1 - tj);
    end
end

X = X';
Z = Z';
init_x = [X(:); 
          T(:);
          E(:);
          U(:); 
          delta(:);
          Z(:); 
          ksi(:);
          W(:);
          V(:)];
end