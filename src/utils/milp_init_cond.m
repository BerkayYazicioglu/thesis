function init_x = milp_init_cond(T_trans, E_trans, e0, w, a, u)
% Calculate init x for the milp solver
n = numel(a);
X = zeros(n, n);
T = zeros(1, n);
U = zeros(1, n); 
delta = zeros(1, n);
P = ones(1, n);
E = zeros(1, n);
E(1) = e0;

% greedy -> highest 'u' value to lowest
[~, idx] = sort(u(2:end), 'descend');
idx = idx + 1;
idx = [1; idx(:)];

for i = 1:n-1
    X(idx(i), idx(i+1)) = 1;
    T(idx(i+1)) = T(idx(i)) + T_trans(idx(i), idx(i+1));
    E(idx(i+1)) = E(idx(i)) - E_trans(idx(i), idx(i+1));

    j = idx(i+1);
    % delta_j=1 => aj <= (1 - Tj)
    if 1 - T(j) >= a(j)
        delta(j) = 1;
    end
    % Uj = wj1 (1 - Tj - aj) + wj2 aj if deltaj = 1
    % Uj = wj3 (aj - 1 + Tj) + wj2 (1 -Tj) if deltaj = 0
    if delta(j) == 1
        U(j) = w(j,1) * (1 - T(j) - a(j)) + w(j,2) * a(j);
    else
        U(j) = w(j,3) * (a(j) - 1 + T(j)) + w(j,2) * (1 - T(j));
    end
    P(i+1) = find(idx == i+1); 
end

X = X';
init_x = [X(:); 
          T(:);
          E(:);
          U(:); 
          delta(:);
          P(:)];
end