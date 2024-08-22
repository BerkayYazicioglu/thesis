%% Policy class for decision making

classdef Policy < handle
    %% Parameters
    properties
        tasks TaskManager
        
        horizon double % horizon relative nodes matrix 
        normalization double; % normalization constant between search and explore
        schedule timetable;
        cooperate_flag = false;

        configs
        data
    end

    methods
        %% Constructor
        function self = Policy(params, tasks, world)
            % params
            % type                -> brute, ga, random
            % tradeoff            -> (constant) search over explore
            % control_horizon     -> (constant) 
            % prediction_horizon  -> (constant)
            % candidate_selection -> 4_frontier, 8_frontier, 
         
            self.tasks = tasks;
            self.configs = params;

            self.normalization = params.normalization;

            self.data.paths = {};
            self.data.frontier = {};
            self.data.analysis = {};
            self.data.cache = {};
            
            self.horizon = [];
            for i = -self.configs.prediction_horizon:self.configs.prediction_horizon
                col = [];
                for j = -self.configs.prediction_horizon:self.configs.prediction_horizon
                    col = [col; j i];
                end
                self.horizon = [self.horizon; col];
            end
        end

        %% Run for the current time step
        function run(self, robot, charger, time)
            arguments
                self Policy
                robot Robot
                charger Charger
                time duration
            end
            % need to replan the trajectory
            self.cooperate_flag = false;
            planned_schedule = self.path_planner(robot, charger);
            planned_schedule.Time = planned_schedule.Time + time;
            if planned_schedule.action(end) ~= "charge"
                planned_schedule = planned_schedule(...
                    1:min(height(planned_schedule), ...
                    self.configs.control_horizon),:);
            end
            if isempty(robot.schedule)
                robot.schedule = planned_schedule;
            else
                robot.schedule = [robot.schedule; planned_schedule];
            end
        end

        %% Path planner
        function planned_schedule = path_planner(self, robot, charger)
            gui = evalin('base', 'gui');   

            self.schedule = timetable();
            % find frontier points
            self.data.frontier = find_candidate_points(robot, self.configs.candidate_selection);
            self.data.frontier = unique(self.data.frontier);
            self.data.paths = {};
            
            % create paths to frontiers
            for i = 1:length(self.data.frontier)
                if self.configs.path_selection == "shortest"
                    path = robot.map.shortestpath(robot.node, ...
                                                  self.data.frontier{i});
                end
                path(1) = [];  
                % trim to maximum prediction horizon 
                path = path(1:min(self.configs.prediction_horizon, length(path)));
                self.data.paths{i} = path;
            end
            self.data.paths(cellfun(@isempty, self.data.paths)) = [];

            n_paths = length(self.data.paths);
            gui.refresh_policy(robot, self.data.paths, 1);
            optimizer_fcn = self.configs.type + "_task_allocator";
           
            % allocate optimal tasks to the paths
            self.data.analysis = cell(1, n_paths);
            self.data.cache = cell(1, n_paths);
            self.data.cache_idx = cell(1, n_paths);
            self.data.n_func = cell(1, n_paths);
            self.data.ratio = cell(1, n_paths);

            trajectories = cell(1, n_paths);
            utilities = zeros(1, n_paths);
            local_minima_flag = true;
            candidate_u = [];

            tic
            for i = 1:n_paths
                gui.refresh_policy(robot, self.data.paths, i);

                [trajectories{i}, utilities(i), analysis] = ...
                    feval(optimizer_fcn, robot, charger, self.data.paths{i});

                candidate_u = [candidate_u; analysis.candidates.u];
                self.data.analysis{i} = analysis.utility;
                self.data.cache{i} = analysis.cache;
                self.data.cache_idx{i} = analysis.cache_idx;
                self.data.n_func{i} = analysis.n_func;
                self.data.ratio{i} = analysis.ratio;

                local_minima_flag = local_minima_flag & analysis.minima_flag;

                gui.print(analysis.log);
            end
            
            % resolve local minima
            if local_minima_flag
                path = local_minima_heuristic(robot);
                self.data.paths = {path};
                [trajectory, utility, analysis] = ...
                        feval(optimizer_fcn, robot, charger, path);
                self.data.analysis = {utility};
                self.data.cache = {analysis.cache};
                self.data.cache_idx = {analysis.cache_idx};
                self.data.n_func = {analysis.n_func};
                self.data.ratio = {analysis.ratio};
                utilities = utility;
                idx = 1;
                gui.print("local minima heuristic| " + analysis.log);
            else
                [~, idx] = max(utilities,[],"all");
                trajectory = trajectories{idx};
            end
            toc
            
            candidate_u_ext = nan(size(trajectory.tasks));
            n_func_ext = nan(size(trajectory.tasks));
            ratio_ext = nan(size(trajectory.tasks));
            fval_ext = nan(size(trajectory.tasks));
            candidate_u_ext(1) = mean(candidate_u);
            n_func_ext(1) = self.data.n_func{idx};
            ratio_ext(1) = self.data.ratio{idx};
            fval_ext(1) = utilities(idx);

            planned_schedule = timetable(trajectory.time, ...
                                         [cellfun(@(x) str2double(x), trajectory.node)], ...
                                         trajectory.action, ...
                                         trajectory.tasks, ...
                                         [arrayfun(@(x) self.configs.type, 1:height(trajectory),'UniformOutput', false)]', ...
                                         candidate_u_ext(:), ...
                                         fval_ext(:), ...
                                         n_func_ext(:), ...
                                         ratio_ext(:), ...
                'VariableNames', {'node', 'action', 'tasks', 'optimizer', 'candidate_u', 'fval', 'n_func', 'ratio'});
            log = sprintf('%s | actions: %s\n', robot.id, strjoin(trajectory.action));
            gui.print(log);
            self.schedule = planned_schedule;
            gui.refresh_policy(robot, self.data.paths, 1);
        end

        %% Capability estimation of a robot performing a task from a given node 
        function c = capability(self, robot, node, direction, task_id, action)
            arguments
                self Policy
                robot Robot
                node string
                direction double
                task_id string
                action string
            end

            task = self.tasks.items(task_id);
            task_node = str2double(task.node);
            [task_x, task_y] = ind2sub(robot.world.grid_dim, task_node);
            [node_x, node_y] = ind2sub(robot.world.grid_dim, str2double(node));
            task_x = task_x - node_x;
            task_y = task_y - node_y;
            c = 0;

            % check if the capability estimation was updated by a
            % measurement
            if ~isempty(task.history)
                id = strsplit(robot.id, "_");
                rf = rowfilter(["robot", "node"]);
                tt = task.history(rf.robot == id(1) & ...
                                  rf.node == string(node), :);
                if ~isempty(tt)
                    c = tt.capability;
                    return
                end
            end
            if action ~= task.type 
                return
            end
            if ~ismember(task.type, [robot.policy.configs.action_list{:}])
                return
            end

            if task.type == "explore"
                for i = 1:length(robot.(task.pi).FoV_rays{direction})
                    line = robot.(task.pi).FoV_rays{direction}{i};
                    line = line(2:end, :);
                    index = find(ismember(line(:, 1:2), [task_x task_y], 'rows'), 1);
                    if ~isempty(index)
                        % check if the line is obstructed on the view
                        epsilon = line(index, 3);
                        % get the valid coordinates
                        line = line(1:index, 1:2) + [node_x node_y];
                        % line = line(all(line(:) >= 1 & ...
                        %                 line(:) <= robot.world.grid_dim, 2), :);
                        line = sub2ind(robot.world.grid_dim, ...
                                       line(:,1), ...
                                       line(:,2));
                        % assuming perfect estimation TODO: add errors
                        terrain = robot.world.environment.Nodes.terrain(line);
                        if max(terrain) == terrain(end)
                            % target is the highest, can be seen
                            c = 1 - epsilon;
                            return
                        elseif any(terrain > robot.height + robot.world.environment.Nodes.terrain(str2double(node)))
                            c = 0;
                            return
                        end
                        % line of sight is not obstructed 
                        c = 1 - epsilon;
                        return
                    end
                end
                   
            elseif task.type == "search"
                fov = robot.(task.pi).FoV{direction};
                % check if the node is in the measurement range
                [~, idx] = ismember([task_x task_y], fov(:,1:2), 'rows');
                if idx == 0
                    c = 0;
                    return
                end
                epsilon = fov(idx, 3);
                c = 1 - epsilon;
            end
        end
    end
end