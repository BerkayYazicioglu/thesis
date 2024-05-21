%% Policy class for decision making

classdef Policy < handle
    %% Parameters
    properties
        tasks TaskManager
        
        horizon double % horizon relative nodes matrix 
        normalization double; % normalization constant between search and explore
        schedule timetable
        counter = 0
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
                    cond_d = (i/self.configs.prediction_horizon)^2 ...
                           + (j/self.configs.prediction_horizon)^2 <= 1;
                    if cond_d 
                        % calculate distance in meters to the origin
                        dist = vecnorm([j i] .* world.cell_size);
                        col = [col; j i];
                    end
                end
                self.horizon = [self.horizon; col];
            end
        end

        %% Run for the current time step
        function run(self, robot, time)
            arguments
                self Policy
                robot Robot
                time duration
            end
            % need to replan the trajectory
            self.cooperate_flag = false;
            planned_schedule = self.path_planner(robot);
            planned_schedule.Time = planned_schedule.Time + time;
            planned_schedule = planned_schedule(...
                1:min(height(planned_schedule), ...
                self.configs.control_horizon),:);

            robot.schedule = [robot.schedule; planned_schedule];
        end

        %% Path planner
        function planned_schedule = path_planner(self, robot)
            gui = evalin('base', 'gui');   

            % find frontier points
            self.data.frontier = find_candidate_points(robot, self.configs.candidate_selection);
            self.data.frontier = unique(self.data.frontier);
            self.data.analysis = {};
            self.data.paths = cell(1, length(self.data.frontier));
            self.data.cache = cell(1, length(self.data.frontier));
            self.data.cache_idx = cell(1, length(self.data.frontier));

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

            n_paths = length(self.data.paths);
            gui.refresh_policy(robot, n_paths, 1);
            optimizer_fcn = self.configs.type + "_task_allocator";
           
            % allocate optimal tasks to the paths
            actions = cell(1, n_paths);
            times = cell(1, n_paths);
            planned_tasks = cell(1, n_paths);
            utilities = zeros(1, n_paths);

            tic
            for i = 1:n_paths
                if ~isempty(self.data.paths{i})
                    gui.refresh_policy(robot, n_paths, i);

                    [actions{i}, utilities(i), times{i}, planned_tasks{i}, analysis] = ...
                        feval(optimizer_fcn, robot, self.data.paths{i});

                    self.data.analysis = [self.data.analysis analysis.utility];
                    self.data.cache{i} = analysis.cache;
                    self.data.cache_idx{i} = analysis.cache_idx;

                    gui.print(analysis.log);
                end
            end
            toc

            [~, idx] = max(utilities,[],"all");
            log = sprintf('%s | actions: %s\n', robot.id, strjoin([actions{idx}{:}]));
                
            P = {};
            A = {};
            T = [];
            for i = 1:length(self.data.paths{idx})
                if actions{idx}{i} == "both"
                    P = [P; self.data.paths{idx}(i); self.data.paths{idx}(i)];
                    A = [A; {"search"; "explore"}];
                    T = [T; times{idx}(i); times{idx}(i)];
                else
                    P = [P; self.data.paths{idx}(i)];
                    A = [A; actions{idx}(i)];
                    T = [T; times{idx}(i)];
                end
            end
            A = cellfun(@(x) x{1}, A, 'UniformOutput', false);

            planned_schedule = timetable(seconds(T), ...
                                         [cellfun(@(x) str2double(x), P)] , ...
                                         convertCharsToStrings(A), ...
                                         arrayfun(@(x) {x}, planned_tasks{idx}), ...
                'VariableNames', {'node', 'action', 'tasks'});
            gui.print(log);
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

            if action ~= task.type 
                return
            end
            if ~ismember(task.type, [robot.policy.configs.action_list{:}])
                return
            end

            if task.type == "explore"
                for i = 1:length(robot.(task.pi).FoV_rays{direction})
                    line = robot.(task.pi).FoV_rays{direction}{i};
                    index = find(ismember(line(2:end, 1:2), [task_x task_y], 'rows'), 1);
                    if ~isempty(index)
                        % check if the line is obstructed on the view
                        epsilon = line(index, 3);
                        % get the valid coordinates
                        line = line(all(line(:, 1:2) >= 1 & ...
                                        line(:, 1:2) <= robot.world.grid_dim, 2), :);
                        line = sub2ind(robot.world.grid_dim, ...
                                       line(2:end,1) + node_x, ...
                                       line(2:end,2) + node_y);
                        % assuming perfect estimation TODO: add errors
                        terrain = robot.world.environment.Nodes.terrain(line);
                        if any(terrain > robot.height + robot.world.environment.Nodes.terrain(task_node))
                            % line of sight is obstructed
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