%% Gui class

classdef Gui < handle
    %% Parameters
    properties
        gui simulation
        mission Mission
        restart_flag = false 

        params
        sim_handles
        data_handles
        data_plot
    end

    methods
        %% Constructor
        function self = Gui(params, simulation_obj, mission, data_plot)
            self.gui = simulation_obj;
            self.mission = mission;
            self.params = params;
            self.data_plot = data_plot;

            self.gui.restart.ButtonPushedFcn = @self.restart_cb;
            self.gui.save.ButtonPushedFcn = @self.save_cb;
            self.gui.robot_select.ValueChangedFcn = @self.select_robot_cb;
            self.gui.data_select.ValueChangedFcn = @self.select_data;

            self.gui.x_init.Limits = [0, mission.world.world_dim(1)];
            self.gui.y_init.Limits = [0, mission.world.world_dim(2)];
         
            self.gui.logger.Value = "";

            robot_ids = cell(1, length(mission.robots));
            for i = 1:length(mission.robots)
                robot_ids{i} = mission.robots{i}.id;
            end
            self.gui.map_select.Items = ["global", robot_ids];
            self.gui.map_select.Value = "global";
            self.gui.robot_select.Items = [robot_ids{:}];
            self.gui.robot_select.Value = robot_ids{1};
            self.gui.data_select.Items = [params.analysis_types{:}];
            self.gui.data_select.Value = params.analysis_types{1};
            self.gui.policy_select.Items = [params.policy_types{:}];
            self.gui.policy_select.Value = params.policy_types{1};
            self.gui.frontier_select.Items = [params.frontier_types{:}];
            self.gui.frontier_select.Value = params.frontier_types{1};
            self.gui.sensor_select.Items = ["visible_sensor" "remote_sensor"];
            self.gui.sensor_select.Value = "visible_sensor";
            self.gui.coordination_select.Items = [params.coordination_types{:}];
            self.gui.coordination_select.Value = mission.settings.coordination.type;
            self.gui.charger_select.Items = [params.charger_types{:}];
            self.gui.charger_select.Value = mission.settings.charger.policy.type;

            self.gui.task.Value = mission.settings.coordination.task;
            self.gui.radius.Value = mission.settings.coordination.radius;
            self.gui.charger_control_horizon.Value = mission.settings.charger.policy.control_horizon;
            self.gui.t_end.Value = mission.settings.t_end;

            id = strsplit(self.gui.robot_select.Value, '_');
            type = id{1};
            idx = str2double(id{2});
            p = self.mission.settings.robots.(type).p_init{idx};
            self.gui.x_init.Value = p{1};
            self.gui.y_init.Value = p{2};
            self.gui.heading.Value = self.mission.settings.robots.(type).heading;
            self.gui.crit_energy.Value = self.mission.settings.robots.(type).crit_energy;
            
            sensor = self.gui.sensor_select.Value;
            self.gui.max_range.Value = self.mission.settings.robots.(type).sensors.(sensor).max_range;
            self.gui.t_s.Value = self.mission.settings.robots.(type).sensors.(sensor).t_s;
            self.gui.d_energy.Value = self.mission.settings.robots.(type).sensors.(sensor).d_energy;
            self.gui.angle.Value = self.mission.settings.robots.(type).sensors.(sensor).angle;
            
            self.gui.tradeoff.Value = self.mission.settings.robots.(type).policy.tradeoff;
            self.gui.prediction_horizon.Value = self.mission.settings.robots.(type).policy.prediction_horizon;
            self.gui.control_horizon.Value = self.mission.settings.robots.(type).policy.control_horizon;
            
            self.gui.policy_select.Value = self.mission.settings.robots.(type).policy.type;
            self.gui.frontier_select.Value = self.mission.settings.robots.(type).policy.candidate_selection;

            cla(self.gui.world,'reset')
            cla(self.data_plot,'reset')
            hold(self.gui.world, 'on')
            hold(self.data_plot, 'on')

            % world handles
            [~, self.sim_handles.terrain] = ...
                contourf(self.gui.world, ...
                         mission.world.X, ...
                         mission.world.Y, ...
                         reshape(mission.world.environment.Nodes.terrain, ...
                                 mission.world.grid_dim), ...
                         'FaceAlpha', 0.7);  

            self.sim_handles.found_victims = ...
                scatter(self.gui.world, ...
                        0, 0, 10, ...
                        'Marker', 'd', ...
                        'MarkerEdgeColor', 'black', ...
                        'MarkerFaceColor', 'flat');

            % mission handles
            c = zeros(size(mission.world.X, 1), ...
                      size(mission.world.X, 2), ...
                      3);
            self.sim_handles.map = ...
                imagesc(self.gui.world, ...
                        [mission.world.X(1,1) mission.world.X(1,end)], ...
                        [mission.world.Y(1,1) mission.world.Y(end,1)], ...
                        c);
            self.sim_handles.search = ...
                scatter(self.gui.world, [], [], 20, ...
                        'Marker', 'd', ...
                        'MarkerEdgeColor', 'magenta', ...
                        'MarkerFaceAlpha', 1);
            self.sim_handles.explore = ...
                scatter(self.gui.world, [], [], 10, ...
                        'Marker', '.', ...
                        'MarkerEdgeColor', 'black', ...
                        'MarkerFaceAlpha', 0.5);
            self.sim_handles.charger = ...
                    plot(self.gui.world, ...
                         mission.world.X(str2double(mission.charger.node)), ...
                         mission.world.Y(str2double(mission.charger.node)), ...
                         'd', 'Color', 'white', ...
                         'MarkerSize', 5, ...
                         'MarkerFaceColor', 'white', ...
                         'MarkerEdgeColor', 'black');
            % robot handles
            for i = 1:length(mission.robots)
                robot = mission.robots{i};
                self.sim_handles.(robot.id).pos = ...
                    plot(self.gui.world, ...
                         mission.world.X(str2double(robot.node)), ...
                         mission.world.Y(str2double(robot.node)), ...
                         '.', 'Color', robot.color, ...
                         'MarkerSize', 20);
                % sensor handles
                self.sim_handles.(robot.id).visible_sensor = ...
                    scatter(self.gui.world, [], [], 7, ...
                            'MarkerFaceAlpha', 0.5, ...
                            'MarkerFaceColor', robot.visible_sensor.color, ...
                            'MarkerEdgeColor', robot.visible_sensor.color);
                self.sim_handles.(robot.id).remote_sensor = ...
                    scatter(self.gui.world, [], [], 7, ...
                            'MarkerFaceAlpha', 0.5, ...
                            'MarkerFaceColor', robot.remote_sensor.color, ...
                            'MarkerEdgeColor', robot.remote_sensor.color);
                % path handles
                self.sim_handles.(robot.id).path = ...
                    plot(self.gui.world, 0, 0, ...
                         'Color', 'black', ...
                         'LineWidth', 1.5);
            end 

            % analysis handles
            % heatmap
            self.data_handles.heatmap.victims = ...
                scatter(self.data_plot, ...
                        [cellfun(@(x) x.location(1), mission.world.victims)], ...
                        [cellfun(@(x) x.location(2), mission.world.victims)], ...
                        10, ...
                        'Marker', 'd', ...
                        'MarkerEdgeColor', 'red', ...
                        'MarkerFaceColor', 'flat'); 
          
            [~, self.data_handles.heatmap.terrain] = ...
                contourf(mission.world.X, ...
                         mission.world.Y, ...
                         reshape(mission.world.environment.Nodes.terrain, ...
                                 mission.world.grid_dim), ...
                         'FaceAlpha', 0);

            c = zeros(size(mission.world.X, 1), ...
                      size(mission.world.X, 2));
            self.data_handles.heatmap.data = ...
                imagesc(self.data_plot, ...
                        [mission.world.X(1,1) mission.world.X(1,end)], ...
                        [mission.world.Y(end,1) mission.world.Y(1,1)], ...
                        c);
            self.data_handles.heatmap.data.AlphaData = 0.7;

            % victim 
            self.data_handles.victim.data = plot(self.data_plot, 0, 0);
            self.data_handles.victim.data.Visible = 'off';

            % utility distribution of the most recent optimization
            self.data_handles.utility.data = plot(self.data_plot, 0, 0);
            self.data_handles.utility.data.Visible = 'off';

            % energy 
            self.data_handles.energy.data = [];
            for i = 1:length(self.mission.robots)
                robot_id = self.mission.robots{i}.id;
                rf = rowfilter("robot");
                self.data_handles.energy.data = [self.data_handles.energy.data 
                    plot(self.data_plot, ...
                        hours(mission.schedule(rf.robot == robot_id, :).Time), ...
                        mission.schedule(rf.robot == robot_id, :).energy)];
                self.data_handles.energy.data(i).Visible = 'off';
            end

            % candidate utilities
            self.data_handles.candidate_utility.data = [];
            for i = 1:length(self.mission.robots)
                robot_id = self.mission.robots{i}.id;
                rf = rowfilter("robot");
                x = [0; hours(mission.schedule(rf.robot == robot_id, :).Time)];
                y = [0; mission.schedule(rf.robot == robot_id, :).candidate_u];
                self.data_handles.candidate_utility.data = [self.data_handles.candidate_utility.data 
                    plot(self.data_plot, x(isfinite(y)), y(isfinite(y)))];
                self.data_handles.candidate_utility.data(i).Visible = 'off';
            end

            % map_area 
            self.data_handles.map_area.data = plot(self.data_plot, 0, 0);
            self.data_handles.map_area.data.Visible = 'off';

            % number of function calls during optimization
            self.data_handles.n_func.data = [];
            for i = 1:length(self.mission.robots)
                robot_id = self.mission.robots{i}.id;
                rf = rowfilter("robot");
                x = [0; hours(mission.schedule(rf.robot == robot_id, :).Time)];
                y = [0; mission.schedule(rf.robot == robot_id, :).n_func];
                self.data_handles.n_func.data = [self.data_handles.n_func.data 
                    plot(self.data_plot, x(isfinite(y)), y(isfinite(y)))];
                self.data_handles.n_func.data(i).Visible = 'off';
            end

            % cache ratios
            self.data_handles.cache_ratio.data = [];
            for i = 1:length(self.mission.robots)
                robot_id = self.mission.robots{i}.id;
                rf = rowfilter("robot");
                x = [0; hours(mission.schedule(rf.robot == robot_id, :).Time)];
                y = [0; mission.schedule(rf.robot == robot_id, :).ratio];
                self.data_handles.cache_ratio.data = [self.data_handles.cache_ratio.data 
                    plot(self.data_plot, x(isfinite(y)), y(isfinite(y)))];
                self.data_handles.cache_ratio.data(i).Visible = 'off';
            end

            % optimization best fval
            self.data_handles.fval.data = [];
            for i = 1:length(self.mission.robots)
                robot_id = self.mission.robots{i}.id;
                rf = rowfilter("robot");
                x = [0; hours(mission.schedule(rf.robot == robot_id, :).Time)];
                y = [0; mission.schedule(rf.robot == robot_id, :).fval];
                self.data_handles.fval.data = [self.data_handles.fval.data 
                    plot(self.data_plot, x(isfinite(y)), y(isfinite(y)))];
                self.data_handles.fval.data(i).Visible = 'off';
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
            sensor = self.gui.sensor_select.Value;

            id = strsplit(selection, '_');
            type = id{1};
            idx = str2double(id{2});
            self.mission.settings.robots.(type).p_init{idx} = {self.gui.x_init.Value, self.gui.y_init.Value};
            self.mission.settings.robots.(type).heading = self.gui.heading.Value;
            self.mission.settings.robots.(type).crit_energy = self.gui.crit_energy.Value;

            self.mission.settings.robots.(type).sensors.(sensor).max_range = self.gui.max_range.Value;
            self.mission.settings.robots.(type).sensors.(sensor).t_s = self.gui.t_s.Value;
            self.mission.settings.robots.(type).sensors.(sensor).d_energy = self.gui.d_energy.Value;
            self.mission.settings.robots.(type).sensors.(sensor).angle = self.gui.angle.Value;

            self.mission.settings.robots.(type).policy.tradeoff = self.gui.tradeoff.Value;
            self.mission.settings.robots.(type).policy.type = self.gui.policy_select.Value;
            self.mission.settings.robots.(type).policy.candidate_selection = self.gui.frontier_select.Value;
            self.mission.settings.robots.(type).policy.prediction_horizon = self.gui.prediction_horizon.Value;
            self.mission.settings.robots.(type).policy.control_horizon =  ...
                min(self.gui.prediction_horizon.Value, ...
                    self.gui.control_horizon.Value);

            self.mission.settings.coordination.type = self.gui.coordination_select.Value;
            self.mission.settings.coordination.task = self.gui.task.Value;
            self.mission.settings.coordination.radius = self.gui.radius.Value;
            self.mission.settings.charger.policy.type = self.gui.charger_select.Value;
            self.mission.settings.charger.policy.control_horizon = self.gui.charger_control_horizon.Value;
            self.mission.settings.t_end = duration(self.gui.t_end.Value);
        end

        %% robot selection callback
        function select_robot_cb(self, app, event)
           val = event.Value;
           id = strsplit(val, '_');
           type = id{1};
           idx = str2double(id{2});
           p = self.mission.settings.robots.(type).p_init{idx};
           self.gui.x_init.Value = p{1};
           self.gui.y_init.Value = p{2};
           self.gui.heading.Value = self.mission.settings.robots.(type).heading;
           self.gui.crit_energy.Value = self.mission.settings.robots.(type).crit_energy;

           sensor = self.gui.sensor_select.Value;
           self.gui.max_range.Value = self.mission.settings.robots.(type).sensors.(sensor).max_range;
           self.gui.t_s.Value = self.mission.settings.robots.(type).sensors.(sensor).t_s;
           self.gui.d_energy.Value = self.mission.settings.robots.(type).sensors.(sensor).d_energy;
           self.gui.angle.Value = self.mission.settings.robots.(type).sensors.(sensor).angle;

           self.gui.tradeoff.Value = self.mission.settings.robots.(type).policy.tradeoff;
           self.gui.prediction_horizon.Value = self.mission.settings.robots.(type).policy.prediction_horizon;
           self.gui.control_horizon.Value = self.mission.settings.robots.(type).policy.control_horizon;

           self.gui.policy_select.Value = self.mission.settings.robots.(type).policy.type;
           self.gui.frontier_select.Value = self.mission.settings.robots.(type).policy.candidate_selection;
        end

        %% data selection callback
        function select_data(self, app, event)
           val = event.Value;
           for i = 1:length(self.params.analysis_types)
               type = self.params.analysis_types{i};
               fields = fieldnames(self.data_handles.(type));
               for ii = 1:length(fields)
                   plots = self.data_handles.(type).(fields{ii});
                   for iii = 1:length(plots)
                       if val == type
                           plots(iii).Visible = 'on';
                       else
                           plots(iii).Visible = 'off';
                       end
                   end
               end
           end
        end

        %% restart callback
        function restart_cb(self, app, event)
            self.restart_flag = true;
            id = strsplit(self.mission.robots{1}.id, '_');
            type = id{1};
            idx = str2double(id{2});
            self.mission.settings.charger.p_init = self.mission.settings.robots.(type).p_init{idx};
        end
        
        %% Refresh policy
        function refresh_policy(self, robot, paths, status)
            if height(robot.policy.schedule) 
                path = [str2num(robot.node); robot.policy.schedule.node];
                delete(self.sim_handles.(robot.id).path);
                self.sim_handles.(robot.id).path = ...
                    plot(self.gui.world, ...
                         self.mission.world.X(path), ...
                         self.mission.world.Y(path), ...
                         'Color', robot.color, ...
                         'LineWidth', 1.5);

                % utility distribution of the most recent optimization
                visibility = self.data_handles.utility.data(1).Visible;
                delete(self.data_handles.utility.data);
                utility = padcat(robot.policy.data.analysis{:})';
                self.data_handles.utility.data = ...
                    plot(self.data_plot, ...
                         [1:length(utility)]', ...
                         utility);
                for i = 1:length(self.data_handles.utility.data)
                    self.data_handles.utility.data(i).Visible = visibility;
                end
            else
                pathsX = cellfun(@(x) self.mission.world.X(...
                    self.mission.world.environment.findnode(x)), ...
                    robot.policy.data.paths, 'UniformOutput', false);
                pathsY = cellfun(@(x) self.mission.world.Y(...
                    self.mission.world.environment.findnode(x)), ...
                    robot.policy.data.paths, 'UniformOutput', false);
                delete(self.sim_handles.(robot.id).path);
                self.sim_handles.(robot.id).path = ...
                    plot(self.gui.world, ...
                         padcat(pathsX{:}), ...
                         padcat(pathsY{:}), ...
                         'Color', robot.color, ...
                         'LineWidth', 0.7);
            end
            
            self.gui.progress_bar.Limits = [1 max(2, length(paths))];
            self.gui.progress_bar.MajorTicks = 1:length(paths);
            self.gui.progress_bar.MinorTicks = [];
            self.gui.progress_bar.Value = status; 
            drawnow
        end

        %% Show current state of the simulation
        function run(self)
            % tables
            mission_table = table(int64(seconds(self.mission.schedule.Time)), ...
                                  [cellfun(@(x) char(x), self.mission.schedule.robot, 'UniformOutput', false)], ...
                                  int64(self.mission.schedule.node), ...
                                  [cellfun(@(x) char(x), self.mission.schedule.action, 'UniformOutput', false)], ...
                                  [cellfun(@(x) char(x), self.mission.schedule.optimizer, 'UniformOutput', false)]);
            self.gui.mission_schedule.Data = table2cell(mission_table);
       
            robot_idx = find([cellfun(@(x) strcmp(x.id, self.gui.robot_select.Value), self.mission.robots)]);
            robot = self.mission.robots{robot_idx};
            if ~isempty(robot.schedule)
                robot_table = table(int64(seconds(robot.schedule.Time)), ...
                                    int64(robot.schedule.node), ...
                                    [cellfun(@(x) char(x), robot.schedule.action, 'UniformOutput', false)], ...
                                    [cellfun(@(x) char(x), robot.schedule.optimizer, 'UniformOutput', false)]);
                self.gui.robot_schedule.Data = table2cell(robot_table);
            end

            c = 0.4 * ones(size(self.mission.world.X, 1), ...
                           size(self.mission.world.X, 2));
            m = cellfun(@str2num, self.mission.map.Nodes.Name);
            c(m) = 0;
            % robots
            set(self.sim_handles.charger, ...
                'XData', self.mission.world.X(str2double(self.mission.charger.node)), ...
                'YData', self.mission.world.Y(str2double(self.mission.charger.node)) ...
                );
            t = self.mission.schedule.Time(end);
            
            stdout = "Time: " + string(t, "hh:mm:ss");
            for i = 1:length(self.mission.robots)
                robot = self.mission.robots{i};
                rf = rowfilter(["node", "robot"]);
                last_action = self.mission.schedule(rf.robot == robot.id & rf.node == str2num(robot.node), :);
                stdout = [stdout; " " + robot.id + "| " + ...
                                  num2str(robot.energy, '%.f') + "% | pos: " + ...
                                  num2str([robot.world.X(str2num(robot.node)) ...
                                           robot.world.Y(str2num(robot.node))], "%.1f ") + ...
                                  " | action: " + last_action.action(end)];
                set(self.sim_handles.(robot.id).pos, ...
                    'XData', self.mission.world.X(str2double(robot.node)), ...
                    'YData', self.mission.world.Y(str2double(robot.node)) ...
                );
                % map
                if robot.id == self.gui.map_select.Value
                    % get only the nonaccessible nodes
                    [bin, ~] = conncomp(robot.map);
                    accessible = 0 * bin;
                    for j = 1:length(self.mission.robots)
                        robot_node = robot.map.findnode(self.mission.robots{j}.node);
                        if robot_node
                            robot_bin = bin(robot_node);
                            accessible = accessible | (bin == robot_bin);
                        end
                    end
                    blocked = cellfun(@str2num, robot.map.Nodes.Name(~accessible));
                    c(blocked) = 0.7;
                end
                set(self.sim_handles.map, 'AlphaData', c);
                % visible sensor
                if isempty(robot.visible_sensor.measurements)
                    set(self.sim_handles.(robot.id).visible_sensor, ...
                        'XData', [], 'YData', []);
                else
                    set(self.sim_handles.(robot.id).visible_sensor, ...
                        'XData', robot.visible_sensor.measurements.coordinates(:, 1), ...
                        'YData', robot.visible_sensor.measurements.coordinates(:, 2));
                end
                % remote sensor
                if isempty(robot.remote_sensor.measurements)
                    set(self.sim_handles.(robot.id).remote_sensor, ...
                        'XData', [], 'YData', []);
                else
                    set(self.sim_handles.(robot.id).remote_sensor, ...
                        'XData', robot.remote_sensor.measurements.coordinates(:, 1), ...
                        'YData', robot.remote_sensor.measurements.coordinates(:, 2));
                end
            end
            self.gui.system.Value = stdout;

            % tasks
            tasks = [self.mission.tasks.items.values];
            e_locs = [tasks([tasks.type] == "explore").location];
            s_locs = [tasks([tasks.type] == "search").location];
            if ~isempty(e_locs)
                e_locs = reshape(e_locs, 2, length(e_locs)/2)';
                set(self.sim_handles.explore, ...
                    'XData', e_locs(:, 1), ...
                    'YData', e_locs(:, 2)); 
            else
                set(self.sim_handles.explore, 'XData', [], 'YData', []); 
            end
            if ~isempty(s_locs)
                s_locs = reshape(s_locs, 2, length(s_locs)/2)';
                set(self.sim_handles.search, ...
                    'XData', s_locs(:, 1), ...
                    'YData', s_locs(:, 2)); 
            else
                set(self.sim_handles.search, 'XData', [], 'YData', []); 
            end

            % analysis
            if ~isempty(self.mission.schedule)
                c = zeros(size(self.mission.world.X, 1), ...
                          size(self.mission.world.X, 2));
                [GC,GR] = groupcounts(self.mission.schedule.node);
                c(GR) = GC;
                c = c ./ max(c,[],'all');
                c = floor(c * 256);
                % robot heatmap count
                set(self.data_handles.heatmap.data, 'CData', flipud(c));

                % victim count plot
                set(self.data_handles.victim.data, ...
                    'XData', seconds(self.mission.schedule.Time), ...
                    'YData', [cellfun(@(x) length(x), self.mission.schedule.Ndet)]); 

                % map area plot
                set(self.data_handles.map_area.data, ...
                    'XData', seconds(self.mission.schedule.Time), ...
                    'YData', self.mission.schedule.map_area); 
             
                for i = 1:length(self.mission.robots)
                    robot_id = self.mission.robots{i}.id;
                    % energy plot
                    rf = rowfilter("robot");
                    set(self.data_handles.energy.data(i), ...
                        'XData', hours(self.mission.schedule(rf.robot == robot_id, :).Time), ...
                        'YData', self.mission.schedule(rf.robot == robot_id, :).energy);
                    % candidate utility plot
                    x = [0; hours(self.mission.schedule(rf.robot == robot_id, :).Time)];
                    y = [0; self.mission.schedule(rf.robot == robot_id, :).candidate_u];
                    set(self.data_handles.candidate_utility.data(i), ...
                        'XData', x(isfinite(y)), ...
                        'YData', y(isfinite(y)));
                    % number of function calls during optimization
                    x = [0; hours(self.mission.schedule(rf.robot == robot_id, :).Time)];
                    y = [0; self.mission.schedule(rf.robot == robot_id, :).n_func];
                    set(self.data_handles.n_func.data(i), ...
                        'XData', x(isfinite(y)), ...
                        'YData', y(isfinite(y)));
                    % cache ratios
                    x = [0; hours(self.mission.schedule(rf.robot == robot_id, :).Time)];
                    y = [0; self.mission.schedule(rf.robot == robot_id, :).ratio];
                    set(self.data_handles.cache_ratio.data(i), ...
                        'XData', x(isfinite(y)), ...
                        'YData', y(isfinite(y)));
                    % optimization best fval
                    x = [0; hours(self.mission.schedule(rf.robot == robot_id, :).Time)];
                    y = [0; self.mission.schedule(rf.robot == robot_id, :).fval];
                    set(self.data_handles.fval.data(i), ...
                        'XData', x(isfinite(y)), ...
                        'YData', y(isfinite(y)));
                end
            end
            axis(self.data_plot, 'tight');
            drawnow
        end
    end
end