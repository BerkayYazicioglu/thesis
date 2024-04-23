%% Mission model

classdef Mission < handle
    %% Parameters
    properties
        world World;
        robots cell = {};
        map graph = graph;
        t duration = seconds(0);
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
                    self.robots{k}.place(cell2mat(r_params.p_init{i}));
                    self.robots{k}.perform("explore", true);
                    self.map = self.robots{k}.update_maps(self.map);
                    k = k+1;
                end
            end

            % data
            self.data.t = [];
            self.data.victim_count = [];
            self.data.robot_locs = cell(size(self.robots));
            self.data.utility = [];
        end

        %% run for the next time step
        function run(self)
            % get the next time step from the robots
            r_times = [];
            for i = 1:length(self.robots)
                r_times = [r_times self.robots{i}.time];
            end
            [new_time, idx] = min(r_times);

            self.map = self.robots{idx}.run(self.map);
            self.t = new_time;
            self.refresh(self.robots{idx});

            % save analysis data and manage found victims
            self.data.t = [self.data.t seconds(self.t)];

            for i = 1:length(self.robots)
                self.data.robot_locs{i} = [self.data.robot_locs{i} str2double(self.robots{i}.node)];
            end

            if ~isempty(self.robots{idx}.remote_sensor.measurements)
                victim_idx = find(self.robots{idx}.remote_sensor.measurements.victim);
                victim_idx = self.robots{idx}.remote_sensor.measurements.victim(victim_idx);
                % new victims found
                if ~isempty(victim_idx)
                     self.N_det = [self.N_det self.world.victims(victim_idx)];
                end
            end
            self.data.victim_count = [self.data.victim_count length( self.N_det)];
            self.data.utility = padcat(self.robots{idx}.policy.data.analysis{:})';
        end
      
        %% refresh settings 
        function refresh(self, robot)
            self.t_end = duration(self.settings.t_end);
            % task manager settings
            self.tasks.configs = self.settings.tasks;
            % robot sensor and policy settings
            id = strsplit(robot.id, '_');
            type = id{1};
            idx = str2double(id{2});
            r_settings = self.settings.robots.(type);
            rs_flag = robot.remote_sensor.max_range ~= ...
                      r_settings.sensors.remote_sensor.max_range;
            vs_flag = robot.visible_sensor.max_range ~= ...
                      r_settings.sensors.visible_sensor.max_range;
            p_flag = robot.policy.configs.prediction_horizon ~= ...
                     r_settings.policy.prediction_horizon;

            if rs_flag
                robot.remote_sensor = Sensor(r_settings.sensors.remote_sensor, ...
                                             robot.world, ...
                                             robot.directions);
            end
            if vs_flag
                robot.visible_sensor = Sensor(r_settings.sensors.visible_sensor, ...
                                              robot.world, ...
                                              robot.directions);
            end
            if p_flag
                r_settings.policy.normalization = robot.remote_sensor.t_s ...
                                                / robot.visible_sensor.t_s;
                robot.policy = Policy(r_settings.policy, self.tasks, self.world);
            end
        end
    end
end


