classdef Gui < handle
    %GUI GUI wrapper for simulation
    properties
        sim simulation;
        mission Mission;
        world World;
        restart_flag = false;

        sim_handles;
        plot_axes;
        plot_handle;
        settings;
    end
    
    methods
        %% Constructor
        function obj = Gui(params, sim_obj, mission, world)
            obj.settings = params;
            obj.sim = sim_obj;
            obj.world = world;
            obj.plot_axes = sim_obj.data;
            obj.init(mission);
        end

        %% Initalize GUI with the given mission 
        function init(obj, mission)
            obj.restart_flag = false;
            delete(obj.sim.world.Children)
            obj.mission = mission; 

            obj.sim.logger.Value = "";
            obj.sim.system.Value = "";
            % plot handles
            obj.sim_handles.world = obj.world.init_gui(obj.sim.world);
            obj.sim_handles.mission = mission.init_gui(obj.sim.world);
            for i = 1:length(mission.robots)
                obj.sim_handles.(mission.robots(i).id) = mission.robots(i).init_gui(obj.sim.world);
                obj.sim_handles.(mission.robots(i).id).map.Visible = 'off';
            end
            obj.sim_handles.mission.charger.map.Visible = 'off';

            obj.sim_handles.world.victims.Visible = 'off';
            obj.sim_handles.world.destruction.Visible = 'off';
            obj.sim_handles.world.population.Visible = 'off';

            % simulation tab
            obj.sim.map_select.Items = ["global", "charger", [mission.robots.id]];
            obj.sim.map_select.Value = "global";
            obj.sim.data_select.Items = [obj.settings.analysis_types{:}];
            obj.sim.data_select.Value = obj.settings.analysis_types{1};
            % settings tab
            obj.sim.t_end.Value = string(mission.t_end);
            obj.sim.mcdm_select.Items = mission.mcdm.key;
            obj.sim.mcdm_select.Value = mission.mcdm.key(1);
            obj.sim.mcdm_value.Value = mission.mcdm.weight(1);
            obj.sim.coordination_radius.Value = mission.coordination_radius;
            obj.sim.x0.Value = mission.q_init{1};
            obj.sim.y0.Value = mission.q_init{2};
            obj.sim.charger_control_horizon.Value = mission.charger.policy.control_horizon;
            obj.sim.task_proximity.Value = mission.charger.policy.task_proximity;
            obj.sim.charger_optimizer.Value = mission.charger.policy.optimizer;
            if mission.prediction_errors
                obj.sim.prediction_errors.Color = [0.39 0.83 0.07];
            else
                obj.sim.prediction_errors.Color = [0.9412 0.9412 0.9412];
            end
            obj.sim.robot_select.Items = [mission.robots.id];
            obj.sim.robot_select.Value = mission.robots(1).id;
            get_robot_settings(obj, mission.robots(1).id);

            % plot handle
            obj.plot_handle = feval(obj.sim.data_select.Value+"_plot", obj);

            % callback functions
            obj.sim.restart.ButtonPushedFcn = @obj.restart_cb;
            obj.sim.map_select.ValueChangedFcn = @obj.select_map_cb;
            obj.sim.data_select.ValueChangedFcn = @obj.select_data;
            % settings tab
            obj.sim.robot_select.ValueChangedFcn = @obj.select_robot_cb;
            obj.sim.sensor_select.ValueChangedFcn = @obj.select_sensor_cb;
            obj.sim.mcdm_select.ValueChangedFcn = @obj.select_mcdm_cb;
        end

        %% Run GUI updates
        function run(obj)
            arrayfun(@(robot) obj.print(robot), obj.mission.robots);
            obj.print(obj.mission.charger);

            system = sprintf('mission time: %s\n\n', string(obj.mission.time));
            for r = 1:length(obj.mission.robots)
                robot = obj.mission.robots(r);
                r_str = robot.id + sprintf('\n %15s', string(robot.time));
                r_str = r_str + sprintf('\n %15.1f %%', string(robot.energy));
                latest_action = find(robot.history.action ~= "none", 1, 'last');
                if isempty(latest_action)
                    latest_action = "none";
                else
                    latest_action = robot.history.action(latest_action);
                end
                r_str = r_str + sprintf('\n %15s\n', latest_action);
                system = system + r_str;
            end
            obj.sim.system.Value = system;

            % verbosity levels
            if obj.settings.verbose == 2
                if ~obj.mission.gui_update_flag
                    return
                end
            elseif obj.settings.verbose == 3
                return;
            end
            obj.mission.update_gui(obj.sim_handles.mission);
            arrayfun(@(robot) robot.update_gui(obj.sim_handles.(robot.id)), obj.mission.robots);

            feval(obj.sim.data_select.Value+"_plot", obj, obj.plot_handle);
           
            % save gif
            if obj.settings.save_gif
                exportgraphics(obj.sim.world,'simulation.gif','Append',true);
            end
            drawnow;
        end

        %% Export all plots to a pdf
        function export_figures(obj, dir)
            plot_file = dir + "/plots.pdf";
            if obj.settings.save_gif
                movefile('simulation.gif', dir);
            else
                obj.sim.world.Title.String = dir;
                exportgraphics(obj.sim.world, plot_file);
            end
            % plot axes
            for i = 1:length(obj.sim.data_select.Items)
                cla(obj.plot_axes, 'reset');
                obj.plot_handle = feval(obj.sim.data_select.Items(i)+"_plot", obj);
                exportgraphics(obj.plot_axes, plot_file, 'Append', true);
            end
        end

        %% Logger
        function print(obj, robot)
            str = robot.msg;
            if strlength(str)
                str = string(obj.mission.time) + " | " + str;
                obj.sim.logger.Value = [obj.sim.logger.Value; str];
                scroll(obj.sim.logger, 'bottom');
            end
            robot.msg = "";
        end

        %% data selection callback
        function select_data(obj, app, event)
            cla(obj.plot_axes, 'reset');
            obj.plot_handle = feval(event.Value+"_plot", obj);
            drawnow
        end

        %% map display selection callback
        function select_map_cb(obj, app, event)
           val = event.Value;
           if ismember(val, [obj.mission.robots.id])
               obj.sim_handles.mission.map.Visible = 'off';
               for i = 1:length(obj.mission.robots)
                   id = obj.mission.robots(i).id;
                   if id == val
                       obj.sim_handles.(id).map.Visible = 'on';
                   else
                       obj.sim_handles.(id).map.Visible = 'off';
                   end
               end
               obj.sim_handles.mission.charger.map.Visible = 'off';
           elseif val == "charger"
               obj.sim_handles.mission.map.Visible = 'off';
               arrayfun(@(r) set(obj.sim_handles.(r.id).map,'Visible','off'), obj.mission.robots);
               obj.sim_handles.mission.charger.map.Visible = 'on';
           else
               obj.sim_handles.mission.map.Visible = 'on';
               arrayfun(@(r) set(obj.sim_handles.(r.id).map,'Visible','off'), obj.mission.robots);
               obj.sim_handles.mission.charger.map.Visible = 'off';
           end
           drawnow
        end

        %% restart callback
        function restart_cb(obj, app, event)
            obj.restart_flag = true;
        end

        %% robot settings selection callback
        function select_robot_cb(obj, app, event)
            robot_id = event.Value;
            get_robot_settings(obj, robot_id);
        end

        %% sensor settings selection callback
        function select_sensor_cb(obj, app, event)
            sensor = event.Value;
            robot_id = obj.sim.robot_select.Value;
            get_sensor_settings(obj, robot_id, sensor);
        end

        %% select mcdm criteria
        function select_mcdm_cb(obj, app, event)
            mcdm_row = event.Value == obj.mission.settings.mcdm.weights.key;
            obj.sim.mcdm_value.Value = obj.mission.settings.mcdm.weights.weight(mcdm_row);
        end
    end
end

