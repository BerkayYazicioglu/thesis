%% Search for best combined strategy from the most recent caches of given robots
% assuming the caches are pruned and simplified

function [trajectories, analysis] = cooperation_optimization(robots, charger, max_iter)
    gui = evalin('base', 'gui');   
    for i = 1:length(robots)
        % recalculate policy if the robots are in control period
        if isnan(robots(i).schedule.candidate_u(1))
            gui.print("cooperation |" + robots(i).id + " policy recalculation");
            robots(i).schedule = timetable();
            robots(i).policy.run(robots(i), charger, robots(i).time);
        end
    end

    caches = [arrayfun(@(x) x.policy.data.cache, robots, 'UniformOutput', false)];
    cache_indices = [arrayfun(@(x) x.policy.data.cache_idx, robots, 'UniformOutput', false)];
    cache_u = [arrayfun(@(x) x.policy.data.analysis', robots, 'UniformOutput', false)];

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
    plots = {};
    if charger.world.params.detail_plots
        plots = {'surrogateoptplot'};
    end
    best_fval = 0;
    best_C = [];
    best_x = [];

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
            best_C = C;
            best_x = x;
        end
    end

    % process the best results
    trajectories = cell(1, length(robots));
    for j = 1:length(robots)
        a = caches{j}{best_C(j)}.Nodes.Name(cache_indices{j}{best_C(j)}(best_x(j)));
        t_idx = caches{j}{best_C(j)}.findnode(caches{j}{best_C(j)}.shortestpath('0', a));
        Times = caches{j}{best_C(j)}.Nodes.t(t_idx);
        tasks = caches{j}{best_C(j)}.Nodes.tau(t_idx);
        a = str2num(char(num2cell(char(a))));
        actions = robots(j).policy.configs.action_list(a(2:end));
        
        if isempty(actions)
            % nowhere to go
            actions = {"charge"};
        elseif caches{j}{best_C(j)}.Nodes.flag(cache_indices{j}{best_C(j)}(best_x(j))) == 2
            actions = [actions {"charge"}];
        end
        actions = [actions{:}];
        path = robots(j).policy.data.paths{best_C(j)};

        if actions(end) == "charge"
            actions(end) = [];
            [~, idx] = min(abs(charger.schedule.Time - Times(end) - robots(j).time));
            charger_node = num2str(charger.schedule.node(idx));
            % find the shortest path to the charger node
            if ~isempty(actions)
                path = path(1:length(actions));
                [p, ~, edge] = robots(j).map.shortestpath(path(end), charger_node);
            else
                path = {};
                [p, ~, edge] = robots(j).map.shortestpath(robot.node, charger_node);
            end
            last_t = Times(end);
            for i = 2:length(p)
                distance = robots(j).world.environment.Edges(edge(i-1),:).Weight;
                Times = [Times(:); last_t + seconds(distance/robots(j).speed)];
                actions = [actions(:); "none"];
                path = [path(:); p{i}];
                last_t = Times(end);
                tasks = [tasks(:); {""}];
            end
            actions(end) = "charge";
            Times(end) = Times(end) + robots(j).charge_s;
        end
        Times = Times(2:end);
        tasks = tasks(2:end);
        % separate planned tasks
        d_tasks = cell(size(tasks));
        for i = 2:length(d_tasks)
            d_tasks{i} = setdiff(tasks{i}, tasks{i-1});
            if isempty(d_tasks{i})
                d_tasks{i} = "";
            end
        end
        d_tasks{1} = tasks{1};

        log = sprintf('%s |cooperation  |actions: %s', robots(j).id, strjoin(actions));
        gui.print(log);

        trajectories{j} = table(Times(:), path(:), actions(:), d_tasks, ...
            'VariableNames', {'time', 'node', 'action', 'tasks'});
    end
    toc
    gui.print("");
end