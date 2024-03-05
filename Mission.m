%% Mission model

classdef Mission < handle
    %% Parameters
    properties
        world World;
        robots cell = {};
        map graph = graph;
        t duration = seconds(0);
        tasks TaskManager;

        N_det uint32 {mustBePositive};
        N_res uint32 {mustBePositive};
        t_end duration; 
    end

    %% Methods
    methods
        function self = Mission(config)
            self.world = World(config.world);
            self.t_end = duration(config.mission.t_end);
            self.tasks = TaskManager(config.tasks, ...
                                     self.world);
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
                    self.robots{k}.perform("explore", true);
                    self.map = self.robots{k}.update_maps(self.map);
                    k = k+1;
                end
            end

            % % validate policy
            % for i = 1:length(self.robots)
            %     x = self.robots{i}.state_space();
            %     u = self.robots{i}.node;
            %     validateFcns(self.robots{i}.policy, x, u, [], self.robots(i));
            % end
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

            disp(toc);
        end
      
        %% plotter
        function plot(self, gui)
            persistent handles
            if nargin > 1
                cla(gui.world,'reset')
                cla(gui.population,'reset')
                
                hold(gui.world,'on')
                hold(gui.population,'on')

                self.world.plot(gui);
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
                gui.robot_select.Items = ["", robot_ids];
                gui.robot_select.Value = "";
                gui.x_init.Limits = [0, self.world.world_dim(1)];
                gui.y_init.Limits = [0, self.world.world_dim(2)];
            end
            self.world.plot();
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
                    % get only the nonaccessible nodes
                    [bin, ~] = conncomp(self.robots{i}.map);
                    accessible = 0 * bin;
                    for j = 1:length(self.robots)
                        robot_node = self.robots{i}.map.findnode(self.robots{j}.node);
                        robot_bin = bin(robot_node);
                        accessible = accessible | ...
                                     (bin == robot_bin);
                    end
                    blocked = cellfun(@str2num, self.robots{i}.map.Nodes.Name(~accessible));
                    c(blocked) = 0.7;
                else
                    continue
                end
            end
            set(handles.map_world, 'AlphaData', c);
            set(handles.system, 'Value', stdout);
        end
    end
end


