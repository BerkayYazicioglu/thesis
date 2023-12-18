%% Mission model

classdef Mission < handle
    %% Parameters
    properties
        world World;
        robots cell = {};
        map graph = graph;
        t duration = seconds(0);
        tasks GlobalTasks;

        N_det uint32 {mustBePositive};
        N_res uint32 {mustBePositive};
        t_end duration; 
    end

    %% Methods
    methods
        function self = Mission(config)
            self.world = World(config.world);
            self.t_end = duration(config.mission.t_end);
            self.tasks = GlobalTasks(config.tasks, ...
                                     self.world, ...
                                     self.map, ...
                                     self.t);
            % populate robots
            k = 1;
            types = fieldnames(config.mission.robots);
            for r = 1:length(types)
                type = types{r};
                r_params = config.mission.robots.(type);
                for i = 1:r_params.count
                    r_params.map_features = config.mission.map_features;
                    self.robots{k} = Robot(r_params, config.sensors, self.world, self.tasks);
                    self.robots{k}.place(cell2mat(r_params.p_init{i}));
                    self.map = self.robots{k}.update_global_map(self.map);
                    self.robots{k}.update_local_map(self.map);
                    k = k+1;
                end
            end
            % populate global task set
            new_tasks = self.tasks.spawn_tasks(self.map, self.t);
            for i = 1:length(new_tasks)
                self.tasks.add_task(new_tasks{i});
            end
        end

        %% run for the next time step
        function run(self)
            % get the next time step from the robots
            tic;

            r_times = [];
            for i = 1:length(self.robots)
                r_times = [r_times self.robots{i}.time];
            end
            [new_time, idx] = min(r_times);

            self.map = self.robots{idx}.run(self.map);
            self.t = new_time;

            % get tasks in the vicinity of the current robot
            % field = self.robots{idx}.visible_sensor.get_all(self.robots{idx}.node, ...
            %                                                self.world);
                
            % for i = 1:length(field)
            %     f = field(i);
            %     if self.tasks{f}.type == "none"
            %         continue
            %     elseif self.tasks{f}.robot == "none"
            %         c = self.tasks{f}.capability(self.robots{idx}, self.robots{idx}.node);
            %         if c >= 0.99
            %            self.tasks{f}.robot = self.robots{idx}.id;
            %            % WIP
            %            self.tasks{f}.type = "none";
            %         end
            %    end 
            % end
            disp(toc);
        end
      
        %% plotter
        function plot(self, gui)
            persistent handles
            if nargin > 1
                self.world.plot(self.t, gui);
                robot_ids = cell(1, length(self.robots));
                for i = 1:length(self.robots)
                    self.robots{i}.plot(gui);
                    robot_ids{i} = self.robots{i}.id;
                end
                self.tasks.plot(gui);
                c = zeros(size(self.world.X, 1), ...
                          size(self.world.X, 2), ...
                          3);
                handles.map_world = imagesc(gui.world, ...
                                            [self.world.X(1,1) self.world.X(1,end)], ...
                                            [self.world.Y(1,1) self.world.Y(end,1)], ...
                                            c);
                handles.system = gui.system;
                handles.map_select = gui.map_select;
                handles.map_select.Items = ["global", robot_ids];
                handles.map_select.Value = "global";
            end
            self.world.plot(self.t);
            stdout = string(self.t, "hh:mm:ss") + " |";
            for i = 1:length(self.robots)
                self.robots{i}.plot();
                stdout = stdout + " " + self.robots{i}.id + " : " + ...
                                        num2str(self.robots{i}.energy) + "% |";
            end

            % tasks
            self.tasks.plot();
           
            % maps
            c = 0.4 * ones(size(self.world.X, 1), ...
                               size(self.world.X, 2));
            m = cellfun(@str2num, self.map.Nodes.Name);
            c(m) = 0;
            for i = 1:length(self.robots)
                if self.robots{i}.id == handles.map_select.Value
                    % get only the accessible nodes
                    [bin, binsize] = conncomp(self.robots{i}.map);
                    accessible = binsize(bin) == max(binsize);
                    accessible = cellfun(@str2num, self.robots{i}.map.Nodes.Name(accessible));
                    c(~accessible) = 0.7;
                else
                    continue
                end
            end
            set(handles.map_world, 'AlphaData', c);
            set(handles.system, 'Value', stdout);
        end
    end
end


