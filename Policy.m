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
            self.data.actions = "";
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
            gui.refresh_policy(robot, self.data.paths, 1);
            optimizer_fcn = self.configs.type + "_task_allocator";
           
            % allocate optimal tasks to the paths
            trajectories = cell(1, n_paths);
            utilities = zeros(1, n_paths);

            tic
            for i = 1:n_paths
                if ~isempty(self.data.paths{i})
                    gui.refresh_policy(robot, self.data.paths, i);

                    [trajectories{i}, utilities(i), analysis] = ...
                        feval(optimizer_fcn, robot, charger, self.data.paths{i});

                    self.data.analysis = [self.data.analysis analysis.utility];
                    self.data.cache{i} = analysis.cache;
                    self.data.cache_idx{i} = analysis.cache_idx;

                    gui.print(analysis.log);
                end
            end
            toc

            [~, idx] = max(utilities,[],"all");
            trajectory = trajectories{idx};
            planned_schedule = timetable(trajectory.time, ...
                                         [cellfun(@(x) str2double(x), trajectory.node)], ...
                                         trajectory.action, ...
                                         trajectory.tasks, ...
                                         [arrayfun(@(x) self.configs.type, 1:height(trajectory),'UniformOutput', false)]', ...
                'VariableNames', {'node', 'action', 'tasks', 'optimizer'});
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