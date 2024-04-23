%% Gui class

classdef Gui < handle
    %% Parameters
    properties
        gui simulation
        mission Mission

        params
        handles
    end

    methods
        %% Constructor
        function self = Gui(params, simulation_obj, mission)
            self.gui = simulation_obj;
            self.mission = mission;
            self.params = params;

            self.gui.restart.ButtonPushedFcn = @self.restart_cb;
            self.gui.save.ButtonPushedFcn = @self.save_cb;
            self.gui.robot_select.ValueChangedFcn = @self.select_robot_cb;
            self.gui.data_select.ValueChangedFcn = @self.select_data;

            self.handles.system = self.gui.system;
            self.handles.logger = self.gui.logger;
            self.handles.map_select = self.gui.map_select;
            self.handles.data_select = self.gui.data_select;
            self.handles.robot_select = self.gui.robot_select;
            self.gui.x_init.Limits = [0, mission.world.world_dim(1)];
            self.gui.y_init.Limits = [0, mission.world.world_dim(2)];
            self.gui.tradeoff.Limits = [0, inf];
            self.gui.heading.Limits = [1, 8];
            self.gui.logger.Value = "";

            robot_ids = cell(1, length(mission.robots));
            for i = 1:length(mission.robots)
                robot_ids{i} = mission.robots{i}.id;
            end
            self.handles.map_select.Items = ["global", robot_ids];
            self.handles.map_select.Value = "global";
            self.handles.robot_select.Items = ["", robot_ids];
            self.handles.robot_select.Value = "";
            self.handles.data_select.Items = [params.analysis_types{:}];
            self.handles.data_select.Value = params.analysis_types{1};

            cla(self.gui.world,'reset')
            cla(self.gui.data,'reset')
            hold(self.gui.world, 'on')
            hold(self.gui.data, 'on')

            % world handles
            self.handles.terrain = ...
                contourf(self.gui.world, ...
                         mission.world.X, ...
                         mission.world.Y, ...
                         reshape(mission.world.environment.Nodes.terrain, ...
                                 mission.world.grid_dim), ...
                         'FaceAlpha', 0.7);  
            v_loc = zeros(length(mission.world.victims), 2);
            v_size = zeros(length(mission.world.victims), 1);
            for v = 1:length(mission.world.victims)
                v_loc(v, :) = mission.world.victims{v}.location;
                v_size(v) = mission.world.victims{v}.size;
            end
            self.handles.victims = ...
                scatter(self.gui.data, ...
                        v_loc(:, 1), v_loc(:, 2), v_size, ...
                        'Marker', 'd', ...
                        'MarkerEdgeColor', 'black', ...
                        'MarkerFaceColor', 'flat'); 
            self.handles.victims.Visible = 'off';
            % mission handles
            c = zeros(size(mission.world.X, 1), ...
                      size(mission.world.X, 2), ...
                      3);
            self.handles.map = ...
                imagesc(self.gui.world, ...
                        [mission.world.X(1,1) mission.world.X(1,end)], ...
                        [mission.world.Y(1,1) mission.world.Y(end,1)], ...
                        c);
            self.handles.search = ...
                scatter(self.gui.world, [], [], 20, ...
                        'Marker', 'd', ...
                        'MarkerEdgeColor', 'magenta', ...
                        'MarkerFaceAlpha', 1);
            self.handles.explore = ...
                scatter(self.gui.world, [], [], 10, ...
                        'Marker', '.', ...
                        'MarkerEdgeColor', 'black', ...
                        'MarkerFaceAlpha', 0.5);
            % analysis handles
            c = zeros(size(mission.world.X, 1), ...
                      size(mission.world.X, 2));
            % robot heatmap count
            self.handles.analysis.(params.analysis_types{1}) = ...
                imagesc(self.gui.data, ...
                        [mission.world.X(1,1) mission.world.X(1,end)], ...
                        [mission.world.Y(end,1) mission.world.Y(1,1)], ...
                        c);
            self.handles.analysis.(params.analysis_types{1}).Parent.YLim = [0 mission.world.Y(end,1)];
            self.handles.analysis.(params.analysis_types{1}).Parent.XLim = [0 mission.world.X(1,end)];

            % victim count plot
            self.handles.analysis.(params.analysis_types{2}) = ...
                plot(self.gui.data, 0, 0);
            self.handles.analysis.(params.analysis_types{2}).Visible = 'off';
            % utility distribution of the most recent optimization
            self.handles.analysis.(params.analysis_types{3}) = ...
                plot(self.gui.data, 0, 0);
            self.handles.analysis.(params.analysis_types{3}).Visible = 'off';
            % robot handles
            for i = 1:length(mission.robots)
                robot = mission.robots{i};
                self.handles.(robot.id).pos = ...
                    plot(self.gui.world, ...
                         mission.world.X(str2double(robot.node)), ...
                         mission.world.Y(str2double(robot.node)), ...
                         '.', 'Color', robot.color, ...
                         'MarkerSize', 20);
                % sensor handles
                self.handles.(robot.id).visible_sensor = ...
                    scatter(self.gui.world, [], [], 7, ...
                            'MarkerFaceAlpha', 0.5, ...
                            'MarkerFaceColor', robot.visible_sensor.color, ...
                            'MarkerEdgeColor', robot.visible_sensor.color);
                self.handles.(robot.id).remote_sensor = ...
                    scatter(self.gui.world, [], [], 7, ...
                            'MarkerFaceAlpha', 0.5, ...
                            'MarkerFaceColor', robot.remote_sensor.color, ...
                            'MarkerEdgeColor', robot.remote_sensor.color);
                % policy handles
                self.handles.(robot.id).policy.frontier = ...
                    scatter(self.gui.world, [], [], 15, ...
                            'Marker', 'o', ...
                            'MarkerFaceColor', 'red', ...
                            'MarkerFaceAlpha', 0.8);
                self.handles.(robot.id).policy.path = ...
                    plot(self.gui.world, 0, 0, ...
                         'Color', 'black', ...
                         'LineWidth', 1.5);

            end 
        end

        %% Logger
        function print(self, str)
            self.gui.logger.Value = [self.gui.logger.Value; str];
            scroll(self.gui.logger, 'bottom');
        end

        %% save settings of the selected robot
        function save_cb(self, app, event)
            selection = self.gui.robot_select.Value;
            if selection == ""
                return
            end
            x = self.gui.x_init.Value;
            y = self.gui.x_init.Value;
            heading = self.gui.heading.Value;
            tradeoff = self.gui.tradeoff.Value;

            id = strsplit(val, '_');
            type = id{1};
            idx = str2double(id{2});
            self.mission.settings.robots.(type).p_init{idx} = {x, y};
            self.mission.settings.robots.(type).heading = heading;
            self.mission.settings.robots.(type).tradeoff = tradeoff;
        end

        %% robot selection callback
        function select_robot_cb(self, app, event)
           val = event.Value;
           if val == ""
               self.gui.x_init.Value = 0;
               self.gui.y_init.Value = 0;
               self.gui.heading = 1;
           else
               id = strsplit(val, '_');
               type = id{1};
               idx = str2double(id{2});
               p = self.mission.settings.robots.(type).p_init{idx};
               self.gui.x_init.Value = p{1};
               self.gui.y_init.Value = p{2};
               self.gui.heading.Value = self.mission.settings.robots.(type).heading;
           end
        end

        %% data selection callback
        function select_data(self, app, event)
           val = event.Value;
           for i = 1:length(self.params.analysis_types)
               type = self.params.analysis_types{i};
               if val == type
                   for ii = 1:length(self.handles.analysis.(type))
                       self.handles.analysis.(type)(ii).Visible = 'on';
                       axis(self.handles.analysis.(type)(ii).Parent, 'tight');
                   end
               else
                   for ii = 1:length(self.handles.analysis.(type))
                       self.handles.analysis.(type)(ii).Visible = 'off';
                   end
               end
           end
        end

        %% restart callback
        function restart_cb(self, app, event)
            new_world =  World(self.mission.world);
            assignin('base', 'world', new_world);
            new_mission = Mission(self.mission.settings, new_world);
            assignin('base', 'mission', new_mission);
        
            cla(self.gui.world,'reset')
            cla(self.gui.data,'reset')
            hold(self.gui.world, 'on')
            hold(self.gui.data, 'on')
        end
        
        %% Refresh policy
        function refresh_policy(self, robot, paths, status)
            if ~isempty(robot.policy.data)
                set(self.handles.(robot.id).policy.path, ...
                    'XData', self.mission.world.X(str2double(robot.policy.data.path)), ...
                    'YData', self.mission.world.Y(str2double(robot.policy.data.path)));
                if isempty(robot.policy.data.frontier)
                    set(self.handles.(robot.id).policy.frontier, ...
                        'XData', [], 'YData', []);
                else
                    p = self.mission.world.environment.findnode(robot.policy.data.frontier);
                    set(self.handles.(robot.id).policy.frontier, ...
                        'XData', self.mission.world.X(p), ...
                        'YData', self.mission.world.Y(p));
                end
            end
            self.gui.progress_bar.Limits = [1 length(paths)];
            self.gui.progress_bar.MajorTicks = 1:length(paths);
            self.gui.progress_bar.MinorTicks = [];
            self.gui.progress_bar.Value = status; 
            drawnow
        end

        %% Show current state of the simulation
        function run(self)
            c = 0.4 * ones(size(self.mission.world.X, 1), ...
                           size(self.mission.world.X, 2));
            m = cellfun(@str2num, self.mission.map.Nodes.Name);
            c(m) = 0;
            % robots
            stdout = string(self.mission.t, "hh:mm:ss") + " |";
            for i = 1:length(self.mission.robots)
                robot = self.mission.robots{i};
                stdout = stdout + " " + robot.id + " :" + ...
                                        num2str(robot.energy, '%.f') + "% |";
                set(self.handles.(robot.id).pos, ...
                    'XData', self.mission.world.X(str2double(robot.node)), ...
                    'YData', self.mission.world.Y(str2double(robot.node)) ...
                );
                % map
                if robot.id == self.handles.map_select.Value
                    % get only the nonaccessible nodes
                    [bin, ~] = conncomp(robot.map);
                    accessible = 0 * bin;
                    for j = 1:length(self.mission.robots)
                        robot_node = robot.map.findnode(self.mission.robots{j}.node);
                        robot_bin = bin(robot_node);
                        accessible = accessible | (bin == robot_bin);
                    end
                    blocked = cellfun(@str2num, robot.map.Nodes.Name(~accessible));
                    c(blocked) = 0.7;
                end
                set(self.handles.map, 'AlphaData', c);
                % visible sensor
                if isempty(robot.visible_sensor.measurements)
                    set(self.handles.(robot.id).visible_sensor, ...
                        'XData', [], 'YData', []);
                else
                    set(self.handles.(robot.id).visible_sensor, ...
                        'XData', robot.visible_sensor.measurements.coordinates(:, 1), ...
                        'YData', robot.visible_sensor.measurements.coordinates(:, 2));
                end
                % remote sensor
                if isempty(robot.remote_sensor.measurements)
                    set(self.handles.(robot.id).remote_sensor, ...
                        'XData', [], 'YData', []);
                else
                    set(self.handles.(robot.id).remote_sensor, ...
                        'XData', robot.remote_sensor.measurements.coordinates(:, 1), ...
                        'YData', robot.remote_sensor.measurements.coordinates(:, 2));
                end
                % policy
                if ~isempty(robot.policy.data)
                    set(self.handles.(robot.id).policy.path, ...
                        'XData', self.mission.world.X(str2double(robot.policy.data.path)), ...
                        'YData', self.mission.world.Y(str2double(robot.policy.data.path)));
                    if isempty(robot.policy.data.frontier)
                        set(self.handles.(robot.id).policy.frontier, ...
                            'XData', [], 'YData', []);
                    else
                        p = self.mission.world.environment.findnode(robot.policy.data.frontier);
                        set(self.handles.(robot.id).policy.frontier, ...
                            'XData', self.mission.world.X(p), ...
                            'YData', self.mission.world.Y(p));
                    end
                end
            end
            set(self.handles.system, 'Value', stdout);

            % tasks
            tasks = [self.mission.tasks.items.values];
            e_locs = [tasks([tasks.type] == "explore").location];
            s_locs = [tasks([tasks.type] == "search").location];
            if ~isempty(e_locs)
                e_locs = reshape(e_locs, 2, length(e_locs)/2)';
                set(self.handles.explore, ...
                    'XData', e_locs(:, 1), ...
                    'YData', e_locs(:, 2)); 
            else
                set(self.handles.explore, 'XData', [], 'YData', []); 
            end
            if ~isempty(s_locs)
                s_locs = reshape(s_locs, 2, length(s_locs)/2)';
                set(self.handles.search, ...
                    'XData', s_locs(:, 1), ...
                    'YData', s_locs(:, 2)); 
            else
                set(self.handles.search, 'XData', [], 'YData', []); 
            end

            % analysis
            c = zeros(size(self.mission.world.X, 1), ...
                      size(self.mission.world.X, 2));
            [GC,GR] = groupcounts([self.mission.data.robot_locs{:}]');
            c(GR) = GC;
            % robot heatmap count
            set(self.handles.analysis.(self.params.analysis_types{1}), ...
                'CData', flipud(c));
            % victim count plot
            set(self.handles.analysis.(self.params.analysis_types{2}), ...
                'XData', self.mission.data.t, ...
                'YData', self.mission.data.victim_count);
            % utility distribution of the most recent optimization
            if ~isempty(self.mission.data.utility)
                visibility = self.handles.analysis.(self.params.analysis_types{3}).Visible;
                set(self.handles.analysis.(self.params.analysis_types{3}), ...
                    'XData', 0, 'YData', 0);
                self.handles.analysis.(self.params.analysis_types{3}) = ...
                plot(self.gui.data, ...
                     [1:size(self.mission.data.utility, 1)]', ...
                     self.mission.data.utility);
            
                for i = 1:length(self.handles.analysis.(self.params.analysis_types{3}))
                    self.handles.analysis.(self.params.analysis_types{3})(i).Visible = visibility;
                end
            end
            axis(self.gui.data, 'tight');
            drawnow
        end
    end
end