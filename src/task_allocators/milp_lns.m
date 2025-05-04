function output = milp_lns(T_trans, E_trans, T_const, E_const, e0, w, a, u, tmax, pred_horizon)

% =========================================================================
max_s_incumbent = 15;
max_s_lns = 5;
size = 6;
max_stall = 5;
max_iter = 10;
% =========================================================================
cache = table({}, {}, {}, {}, [], ...
    'VariableNames', {'x', 't', 'e', 'u', 'u_total'});
cache_idx = 0;

% create an incumbent solution 
[model, params, variables] = milp(T_trans, E_trans, T_const, E_const, e0, w, a, tmax, pred_horizon);
params.TimeLimit = max_s_incumbent;
params.MIPFocus = 1; 
params.outputflag = 1;
result = gurobi(model, params);
if isfield(result, 'pool')
    for i = 1:length(result.pool)
        x_idx = extract_milp_path(result.pool(i).xn, variables);
        if length(x_idx) <= 1
            continue
        end
        t_result = result.pool(i).xn(variables.T(:))';
        e_result = result.pool(i).xn(variables.E(:))';
        w_result = result.pool(i).xn(variables.W(:))';
        cache = [cache; {{x_idx}, ...
                         {t_result(x_idx)}, ...
                         {e_result(x_idx)}, ...
                         {w_result(x_idx)}, ...
                         result.pool(i).objval}];
    end
    x_idx = extract_milp_path(result.x, variables);
    if length(x_idx) > 1
        t_result = result.x(variables.T(:))';
        e_result = result.x(variables.E(:))';
        w_result = result.x(variables.W(:))';
        cache = [cache; {{x_idx}, ...
                         {t_result(x_idx)}, ...
                         {e_result(x_idx)}, ...
                         {w_result(x_idx)}, ...
                         result.objval}];
        cache_idx = height(cache);
    end
else
    disp('gurobi couldnt find a feasible solution, generating initial conditions');
    result.x = milp_init_cond(T_trans, E_trans, T_const, E_const, e0, w, a, pred_horizon, u);
    result.objval = sum(result.x(variables.W(:)));
end

params.TimeLimit = max_s_lns;

% employ large neighborhood search
if pred_horizon > size
    n_unchanged = pred_horizon - size;
    iter = 1;
    stall = 1;

    % start iterations
    while iter <= max_iter
        visited = find(result.x(variables.V(:)));
        visited = visited(2:end);
        weights = result.x(variables.W(visited));
        weights = weights + 0.01;
        weights = weights / sum(weights);
        unchanged = datasample(visited , n_unchanged , 'Replace',false , 'Weights', weights);
        
        model.lb(variables.V(:)) = 0;
        model.ub(variables.V(:)) = 1;
        model.lb(variables.V(1)) = 1;
        model.lb(variables.V(unchanged)) = 1;

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
                x_idx = extract_milp_path(new_result.pool(i).xn, variables);
                if length(x_idx) <= 1
                    continue
                end
                t_result = new_result.pool(i).xn(variables.T(:))';
                e_result = new_result.pool(i).xn(variables.E(:))';
                w_result = new_result.pool(i).xn(variables.W(:))';
                cache = [cache; {{x_idx}, ...
                                 {t_result(x_idx)}, ...
                                 {e_result(x_idx)}, ...
                                 {w_result(x_idx)}, ...
                                 new_result.pool(i).objval}];
            end
            if length(x_idx) > 1
                x_idx = extract_milp_path(new_result.x, variables);
                t_result = new_result.x(variables.T(:))';
                e_result = new_result.x(variables.E(:))';
                w_result = new_result.x(variables.W(:))';
                cache = [cache; {{x_idx}, ...
                                 {t_result(x_idx)}, ...
                                 {e_result(x_idx)}, ...
                                 {w_result(x_idx)}, ...
                                 new_result.objval}];
            end
        else
            new_result.objval = result.objval;
        end
        if new_result.objval > result.objval 
            result = new_result;
            cache_idx = height(cache);
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
output.cache = cache;
output.x = extract_milp_path(result.x, variables);
output.t = result.x(variables.T(output.x))';
output.e = result.x(variables.E(output.x))';
output.u = result.x(variables.W(output.x))';
output.u_total = result.objval;

output.cache_idx = cache_idx;
end