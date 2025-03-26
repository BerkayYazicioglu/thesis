function output = milp_lns(T_trans, E_trans, T_const, E_const, e0, w, a, u)

% =========================================================================
max_work_limit = 5;
size = 5;
max_stall = 8;
max_iter = 20;
random_ratio = 0.5;
% =========================================================================
cache = table({}, {}, {}, {}, [], ...
    'VariableNames', {'x', 't', 'e', 'u', 'u_total'});

% create an incumbent solution 
[model, params, variables] = milp(T_trans, E_trans, T_const, E_const, e0, w, a);
params.WorkLimit = max_work_limit;
params.MIPFocus = 1; 
params.outputflag = 1;
result = gurobi(model, params);
if isfield(result, 'pool')
    for i = 1:length(result.pool)
        [~, x_idx] = sort(result.pool(i).xn(variables.P(:)));
        cache = [cache; {{x_idx'}, ...
                         {result.pool(i).xn(variables.T(:))'}, ...
                         {result.pool(i).xn(variables.E(:))'}, ...
                         {result.pool(i).xn(variables.W(:))'}, ...
                         result.pool(i).objval}];
    end
else
    result.x = milp_init_cond(T_trans, E_trans, e0, w, a, u);
    result.objval = sum(result.x(variables.W(:)));
    [~, x_idx] = sort(result.x(variables.P(:)));
    cache = [cache; {{x_idx'}, ...
                     {result.x(variables.T(:))'}, ...
                     {result.x(variables.E(:))'}, ...
                     {result.x(variables.W(:))'}, ...
                     result.objval}];
end

params.WorkLimit = 100;

% employ large neighborhood search
if variables.n > size + 1
    n_random = floor(size * random_ratio);
    n_worst = size - n_random;
    iter = 1;
    stall = 1;

    % start iterations
    while iter <= max_iter 
        [~, worst_idx] = sort(result.x(variables.W(:)), 'ascend');
        change = worst_idx(2:n_worst+1);
        random_candidates = setdiff(2:variables.n, change);
        random_idx = randperm(numel(random_candidates), n_random);
        if ~isempty(random_idx)
            change = [change; random_candidates(random_idx)'];
        end
        % create constraints to fix the unchanged variables
        unchanged = setdiff(2:variables.n, change);
        X = reshape(result.x(variables.X(:)), [variables.n variables.n]);
        model.lb(variables.X(:)) = 0;
        model.ub(variables.X(:)) = 1;
        for i = 1:length(unchanged)
            Xcol = X(:, unchanged(i));
            model.lb(variables.X(find(Xcol), unchanged(i))) = 1;
        end
        % solve the reduced model
        new_result = gurobi(model, params);
        if isfield(new_result, 'pool')
            for i = 1:length(new_result.pool)
                P_result = new_result.pool(i).xn(variables.P(:));
                U_result = new_result.pool(i).xn(variables.W(:));
                T_result = new_result.pool(i).xn(variables.T(:));
                E_result = new_result.pool(i).xn(variables.E(:));
                [~, x_idx] = sort(P_result);
                cache = [cache; {{x_idx'}, ...
                                 {T_result'}, ...
                                 {E_result'}, ...
                                 {U_result'}, ...
                                 new_result.pool(i).objval}];
            end
        else
            new_result.objval = result.objval;
        end
        if new_result.objval > result.objval 
            result = new_result;
            stall = 1;
        else
            stall = stall + 1;
        end
        iter = iter + 1;
        if stall == max_stall
            break
        end
    end
end

%% Gather results
[~, idx] = max(cache.u_total);

output.cache = cache;
output.x = cache.x{idx};
output.t = cache.t{idx};
output.e = cache.e{idx};
output.u = cache.u{idx};
output.u_total = cache.u_total(idx);
end