%% Mission model

classdef Mission < handle
    %% Parameters
    properties
        world World;
        robots cell = {};
        charger Charger;
        map graph = graph;
        schedule timetable;
        t_end duration; 
        tasks TaskManager;

        N_det cell;

        data 
        settings;
    end

    %% Methods
    methods
        function self = Mission(config, world)
            self.settings = config;
            self.world = world;
            self.t_end = duration(config.t_end);
            self.tasks = TaskManager(config.tasks, self.world);
            % populate robots
            k = 1;
            types = fieldnames(config.robots);
            for r = 1:length(types)
                type = types{r};
                r_params = config.robots.(type);
                for i = 1:r_params.count
                    r_params.map_features = config.map_features;
                    r_params.id = r_params.id + "_" + num2str(i);
                    self.robots{k} = Robot(r_params, self.world, self.tasks);
                    self.map = self.robots{k}.place(seconds(0), ...
                        cell2mat(r_params.p_init{i}), ...
                        "explore", ...
                        self.map);
                    k = k+1;
                end
            end

            self.charger = Charger(config.charger, self.world);

            self.schedule = ...
                timetable(seconds(0) * ones(length(self.robots), 1), ...
                          [cellfun(@(x) x.id, self.robots)]', ...
                          [cellfun(@(x) str2num(x.node), self.robots)]', ...
                          [arrayfun(@(x) "none", 1:length(self.robots))]', ...
                          [arrayfun(@(x) {}, 1:length(self.robots), 'UniformOutput', false)]', ...
                          [arrayfun(@(x) {}, 1:length(self.robots), 'UniformOutput', false)]', ...
                          [arrayfun(@(x) "placement", 1:length(self.robots), 'UniformOutput', false)]', ...
                          'VariableNames', {'robot', 'node', 'action', 'tasks', 'Ndet', 'optimizer'});
        end

        %% run for the next time step
        function run(self)
            % refresh settings
            cellfun(@(x) self.refresh(x), self.robots);
            self.charger.policy = self.settings.charger.policy.type;
            self.charger.control_horizon = self.settings.charger.policy.control_horizon;
            
            % calculate new policy if needed
            flags = [cellfun(@(x) isempty(x.schedule), self.robots)];
            cellfun(@(x) x.policy.run(x, self.charger, x.time), self.robots(flags));

            % get the next time step from the robots
            r_times = [cellfun(@(x) x.schedule.Time(1), self.robots)];
            [new_time, idx] = min(r_times);

            % check for conflicts to resolve
            if length(self.robots) > 1
                pos_idx = [cellfun(@(x)str2double(x.node), self.robots)];
                pos = [self.world.X(pos_idx); self.world.Y(pos_idx)];
                if self.settings.coordination.type == "radius"
                    flags = vecnorm(pos - pos(:,idx)) <= self.settings.coordination.radius;
                elseif self.settings.coordination.type == "task"
                    % find the overlap between the planned tasks
                    planned_tasks = [cellfun(@(x) x.schedule.tasks{:}, self.robots)];
                    planned_tasks = [cellfun(@(x) {x(~cellfun('isempty',x))}, planned_tasks)];
                    for i = 1:length(self.robots)
                        U = union(planned_tasks{i}, planned_tasks{idx});
                        I = intersect(planned_tasks{i}, planned_tasks{idx});
                        task_ratio = 0;
                        if ~isempty(U)
                            task_ratio = numel(I) / numel(U);
                        end
                        flags(i) = task_ratio >= self.settings.coordination.task;
                    end
                end
                flags(idx) = 0;
                if any(flags)
                    flags(idx) = 1;
                    % resolve conflicts using updated optimization caches
                    flagged_robots = [self.robots{find(flags)}];
                    if ~all([arrayfun(@(x) x.policy.cooperate_flag, flagged_robots)])
                        [planned_schedules, analysis] = cooperation_optimization(flagged_robots, 75);
                        if any(cellfun(@(x) isempty(x), cell(1,3)))
                            for i = 1:length(flagged_robots)
                                flagged_robots(i).schedule = planned_schedules{i};
                            end
                        end
                    end
                    r_times = [cellfun(@(x) x.schedule.Time(1), self.robots)];
                    [new_time, idx] = min(r_times);
                end
            end
            
            charger_action = self.charger.run(new_time);
            r_times = [cellfun(@(x) seconds(x.schedule.Time'), self.robots, 'UniformOutput', false)];
            if height(self.charger.schedule) == 0
                self.charger.path_planner(seconds(max([r_times{:}])), ...
                                          self.map, ...
                                          self.tasks);
            end
            if height(charger_action)
                self.schedule = [self.schedule; 
                    timetable(charger_action.Time, ...
                              [arrayfun(@(x) self.charger.id, 1:height(charger_action))]', ...
                              charger_action.node, ...
                              [arrayfun(@(x) "none", 1:height(charger_action))]', ...
                              [arrayfun(@(x) {}, 1:height(charger_action), 'UniformOutput', false)]', ...
                              [arrayfun(@(x) self.N_det, 1:height(charger_action), 'UniformOutput', false)]', ...
                              [arrayfun(@(x) "CoM", 1:height(charger_action))]', ...
                              'VariableNames', {'robot', 'node', 'action', 'tasks', 'Ndet', 'optimizer'})];
            end
            
            % the next robot to run is found
            robot = self.robots{idx};
            new_schedule = addvars(robot.schedule(1,:), robot.id, 'Before', 1, 'NewVariableNames', 'robot');
            self.map = self.robots{idx}.run(self.map);
            % save analysis data and manage found victims
            if ~isempty(self.robots{idx}.remote_sensor.measurements)
                victim_idx = find(self.robots{idx}.remote_sensor.measurements.victim);
                victim_idx = self.robots{idx}.remote_sensor.measurements.victim(victim_idx);
                % new victims found
                if ~isempty(victim_idx)
                     self.N_det = [self.N_det self.world.victims(victim_idx)];
                end
            end
            self.schedule = [self.schedule; 
                addvars(new_schedule, {self.N_det}, ...
                'NewVariableNames', {'Ndet'}, 'Before', width(new_schedule))];
        end
      
        %% refresh settings 
        function refresh(self, robot)
            log = "";
            id = strsplit(robot.id, '_');
            type = id{1};

            % robot settings
            r_settings = self.settings.robots.(type);
            if robot.params.crit_energy ~= r_settings.crit_energy
                robot.params.crit_energy = r_settings.crit_energy;
                robot.crit_energy = robot.params.crit_energy;
                log = log + " crit_energy |"; 
            end
            
            % sensor settings
            if ~isequal(robot.remote_sensor.params, r_settings.sensors.remote_sensor)
                robot.remote_sensor = Sensor(r_settings.sensors.remote_sensor, ...
                                             robot.world, ...
                                             robot.directions);
                robot.schedule = timetable();
                log = log + " remote sensor |";
            end
            if ~isequal(robot.visible_sensor.params, r_settings.sensors.visible_sensor)
                robot.visible_sensor = Sensor(r_settings.sensors.visible_sensor, ...
                                             robot.world, ...
                                             robot.directions);
                robot.schedule = timetable();
                log = log + " visible sensor |";
            end

            % policy settings
            r_settings.policy.normalization = robot.remote_sensor.t_s / robot.visible_sensor.t_s;
            robot.policy.configs.normalization = r_settings.policy.normalization;
            if ~isequal(robot.policy.configs, r_settings.policy)
                robot.policy = Policy(r_settings.policy, self.tasks, self.world);
                robot.schedule = timetable();
                log = log + " policy |";
            end

            if strlength(log)
                gui = evalin('base', 'gui');
                gui.print(robot.id + " |" + log);
            end
        end
    end
end