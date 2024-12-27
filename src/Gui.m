classdef Gui < handle
    %GUI GUI wrapper for simulation
    properties
        sim simulation;
        mission Mission;
        world World;
        restart_flag = false;

        sim_handles;
        plot_handle;
        plot_axes;
        param_cache;
        settings;
    end
    
    methods
        %% Constructor
        function obj = Gui(params, sim_obj, mission, world)
            obj.settings = params;
            obj.sim = sim_obj;
            obj.world = world;
            obj.plot_axes = axes();
            obj.init(mission);
        end

        %% Initalize GUI with the given mission 
        function init(obj, mission)
            obj.restart_flag = false;
            delete(obj.sim.world.Children)
            % save parameter cache
            obj.param_cache = package_mission_settings(mission);
            obj.mission = mission; 
            % plot handles
            obj.sim_handles.world = obj.world.init_gui(obj.sim.world);
            obj.sim_handles.mission = mission.init_gui(obj.sim.world);
            for i = 1:length(mission.robots)
                obj.sim_handles.(mission.robots(i).id) = mission.robots(i).init_gui(obj.sim.world);
                obj.sim_handles.(mission.robots(i).id).map.Visible = 'off';
            end

            obj.sim_handles.world.victims.Visible = 'off';
            obj.sim_handles.world.destruction.Visible = 'off';
            obj.sim_handles.world.population.Visible = 'off';

            % simulation tab
            obj.sim.map_select.Items = ["global", [mission.robots.id]];
            obj.sim.map_select.Value = "global";
            obj.sim.data_select.Items = [obj.settings.analysis_types{:}];
            obj.sim.data_select.Value = obj.settings.analysis_types{1};
            % settings tab
            obj.sim.t_end.Value = string(mission.t_end);
            obj.sim.mcdm_select.Items = mission.mcdm.weights.key;
            obj.sim.mcdm_select.Value = mission.mcdm.weights.key(1);
            obj.sim.mcdm_value.Value = mission.mcdm.weights.weight(1);
            obj.sim.charger_select.Items = [obj.settings.charger_types{:}];
            obj.sim.charger_select.Value = mission.settings.charger.policy.optimizer;
            obj.sim.preprocessing_select.Items = [obj.settings.preprocessing_types{:}];
            obj.sim.optimizer_select.Items = [obj.settings.optimizer_types{:}];
            obj.sim.robot_select.Items = [mission.robots.id];
            obj.sim.robot_select.Value = mission.robots(1).id;
            get_robot_settings(obj, mission.robots(1).id);

            sim_size = obj.world.size();
            obj.sim.x_init.Limits = [1, sim_size(1)];
            obj.sim.y_init.Limits = [1, sim_size(2)];
            obj.sim.prediction_errors.BackgroundColor = [0.9412 0.9412 0.9412];
            obj.sim.prediction_errors.Value = 0;

            % plot handle
            obj.plot_handle = feval(obj.sim.data_select.Value+"_plot", obj);

            % callback functions
            obj.sim.restart.ButtonPushedFcn = @obj.restart_cb;
            obj.sim.map_select.ValueChangedFcn = @obj.select_map_cb;
            obj.sim.data_select.ValueChangedFcn = @obj.select_data;
            % settings tab
            obj.sim.save.ButtonPushedFcn = @obj.save_cb;
            obj.sim.robot_select.ValueChangedFcn = @obj.select_robot_cb;
            obj.sim.sensor_select.ValueChangedFcn = @obj.select_sensor_cb;
            obj.sim.prediction_errors.ValueChangedFcn = @obj.toggle_errors_cb;
            obj.sim.mcdm_select.ValueChangedFcn = @obj.select_mcdm_cb;
        end

        %% Run GUI updates
        function run(obj)
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
                clf(obj.plot_axes.Parent, 'reset');
                obj.plot_axes = axes();
                obj.plot_handle = feval(obj.sim.data_select.Items(i)+"_plot", obj);
                exportgraphics(obj.plot_axes, plot_file, 'Append', true);
            end
        end

        %% Logger
        function print(obj, str)
            obj.sim.logger.Value = [obj.sim.logger.Value; str];
            scroll(obj.sim.logger, 'bottom');
        end

        %% data selection callback
        function select_data(obj, app, event)
            clf(obj.plot_axes.Parent, 'reset');
            obj.plot_axes = axes();
            obj.plot_handle = feval(event.Value+"_plot", obj);
            drawnow
        end

         %% save settings of the selected robot
        function save_cb(obj, app, event)
            save_current_settings(obj);
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
           else
               obj.sim_handles.mission.map.Visible = 'on';
               arrayfun(@(r) set(obj.sim_handles.(r.id).map,'Visible','off'), obj.mission.robots);
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

        %% toggle prediction errors callback
        function toggle_errors_cb(obj, app, event)
            val = event.Value;
            if val
                app.BackgroundColor = [0.39 0.83 0.07];
                obj.mission.prediction_errors = true;
            else
                app.BackgroundColor = [0.9412 0.9412 0.9412];
                obj.mission.prediction_errors = false;
            end
        end

        %% select mcdm criteria
        function select_mcdm_cb(obj, app, event)
            mcdm_row = event.Value == obj.param_cache.mcdm.weights.key;
            obj.sim.mcdm_value.Value = obj.param_cache.mcdm.weights.weight(mcdm_row);
        end
    end
end

