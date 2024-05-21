%% Search for best combined strategy from the most recent caches of given robots
% assuming the caches are pruned and simplified

function [P, A, analysis] = cooperation_optimization(robots, max_iter)
    % recalculate all path planning if not at the starting node
    flags = [arrayfun(@(x) convertCharsToStrings(x.node) ~= ...
                           convertCharsToStrings(x.policy.data.start_node), ...
                      robots)];
    flagged_robots = robots(flags);
    for i = 1:length(flagged_robots)
        flagged_robots(i).policy.run(flagged_robots(i), true);
    end

    caches = [arrayfun(@(x) x.policy.data.cache, robots, 'UniformOutput', false)];
    cache_indices = [arrayfun(@(x) x.policy.data.cache_idx, robots, 'UniformOutput', false)];
    cache_u = [arrayfun(@(x) x.policy.data.analysis, robots, 'UniformOutput', false)];

    %% fitness function for multiple caches
    % 1 -> no action
    % 2 -> explore
    % 3 -> search
    % 4 -> both 
    % C -> n_robots length array of respective cache indices
    % X -> n_robots length array of respective cache node indices
    function u = fitness(X, C)
        n_robots = length(C);
        goal_str = cell(1, n_robots);
        goal_paths = cell(1, n_robots);
        goal_times = cell(1, n_robots);
        traverse_idx = 2 * ones(1, n_robots);
        
        for ii = 1:n_robots
            goal_str(ii) = caches{ii}{C(ii)}.Nodes.Name(cache_indices{ii}{C(ii)}(X(ii)));
            goal_paths{ii} = caches{ii}{C(ii)}.findnode(...
                caches{ii}{C(ii)}.shortestpath('0', goal_str(ii)));
            times = caches{ii}{C(ii)}.Nodes.t(goal_paths{ii});
            goal_times{ii} = [seconds(times); inf];
        end

        Tau = [""];
        u = 0;
        end_flags = zeros(1, n_robots);
        while sum(end_flags) < n_robots
            times = [arrayfun(@(x)goal_times{x}(traverse_idx(x)), ...
                     1:n_robots)];
            [~, ii] = min(times);
            node_idx = goal_paths{ii}(traverse_idx(ii));
            tau = caches{ii}{C(ii)}.Nodes.tau(node_idx);
            u_tau = caches{ii}{C(ii)}.Nodes.u_tau(node_idx);
            % remove empty
            tau{:}(cellfun(@isempty, tau{:})) = [];
            [performed, p_idx] = setdiff(tau{:}, Tau);
            if ~isempty(p_idx)
                for kk = 1:length(performed)
                    u = u + u_tau{:}(p_idx(kk));
                end
                Tau = [Tau; performed(:)];
            end
            traverse_idx(ii) = traverse_idx(ii) + 1;
            if traverse_idx(ii) > length(goal_str{ii})
                traverse_idx(ii) = length(goal_times{ii});
                end_flags(ii) = true;
            end
        end
        u = -u;
    end

    %% get the candidate caches for each robot
    C_indices = cell(1, length(robots));
    for i = 1:length(C_indices)
        n_cache = length(cache_u{i});
        k = max(3, ceil(n_cache / 2));
        max_utilities = cellfun(@(x)max(x), cache_u{i});
        [~, C_indices{i}] = maxk(max_utilities, k);
    end

    % get all candidate cache combinations and brute force search caches
    C_combinations = combinations(C_indices{:});
    plots = {'surrogateoptplot'};
    %plots = {};
    best_fval = 0;
    best_x = [];
    best_c = [];

    A = cell(1, length(robots));
    P = cell(1, length(robots));

    % analysis
    analysis = table({{}}, {{}}, 0,  ...
        'VariableNames',{'paths', 'actions', 'utility'});

    tic
    for i = 1:height(C_combinations)
        C = table2array(C_combinations(i, :));
        % surrogate optimization
        surrogate_fcn = @(x) fitness(x, C);   
        lb = ones(1, length(robots));
        ub = zeros(1, length(robots));
        for j = 1:length(robots)
            ub(j) = length(cache_indices{j}{C(j)});
        end
        srg_options = optimoptions('surrogateopt', ... 
            'PlotFcn', plots, ...
            'InitialPoints', ones(1, length(robots)), ...
            'MaxFunctionEvaluations', max_iter, ...
            'Display', 'none');
        [x, fval, ~, output, trials] = ...
            surrogateopt(surrogate_fcn, ...
                         lb, ...
                         ub, ...
                         1:length(robots), ...
                         srg_options);
        % analysis
        actions = cell(1, length(robots));
        paths = cell(1, length(robots));
        for j = 1:length(robots)
            robots(j).policy.cooperate_flag = true;
            actions{j} = caches{j}{C(j)}.Nodes.Name( ...
                cache_indices{j}{C(j)}(x(j)));
            paths{j} = C(j);
        end
        analysis = [analysis; {{paths}, {actions}, -fval}];

        % select best
        if fval <= best_fval
            best_fval = fval; 
            for j = 1:length(robots)
                a = caches{j}{C(j)}.Nodes.Name( ...
                    cache_indices{j}{C(j)}(x(j)));
                a = str2num(char(num2cell(char(a))));
                A{j} = robots(j).policy.configs.action_list(a(2:end));
                P(j) = robots(j).policy.data.paths(C(j));
            end
        end
    end
    toc
end