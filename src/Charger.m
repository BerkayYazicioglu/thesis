classdef Charger < handle
    %CHARGER Charger robot without any task capabilities
    
    properties
        % params fields
        id string;
        color string;
        max_step double {mustBeReal, mustBeNonnegative}; % m 
        speed double {mustBeReal, mustBeNonnegative}; % m/s
        policy; % struct for policy settings
        q_init cell;

        node string;
        time duration;
        schedule timetable;
        idle logical = false;
        msg string = "";

        world World;
        mission Mission;
        history timetable;
        pp_outputs;
        opt_results;
    end
    
    methods
        %% Constructor
        function obj = Charger(params, world, mission)
            %CHARGER params (struct) ->
            %   id          : robot id
            %   color       : plotting color
            %   max_step    : maximum step increase the robot can climb (m)
            %   speed       : travel speed m/s
            %   policy      : policy settings

            obj.world = world;
            obj.mission = mission;
            
            obj.id = params.id;
            obj.color = params.color; 
            obj.max_step = params.max_step;
            obj.speed = params.speed;
            obj.policy = params.policy;
            obj.opt_results = struct();
            obj.pp_outputs = dictionary();
            
            % place the robot 
            obj.q_init = mission.q_init;
            obj.node = world.get_id(obj.q_init{1}, obj.q_init{2});
            obj.time = seconds(0); 
            obj.time.Format = 'hh:mm:ss';
            obj.schedule = timetable(obj.time, obj.node, ...
                'VariableNames', {'node'});

            % history
            obj.history = timetable(obj.time, obj.node,...
                'VariableNames', {'node'});
        end

        %% Charge a robot
        function charge(obj, robot)
            if obj.node ~= robot.node
                warning("Robot and charger need to be on the same node");
                robot.node = obj.node;
            end
            de = 100 - robot.energy;
            robot.schedule = timetable(robot.time + seconds(de/robot.charge_per_s), ...
                         obj.node, "charge_done", 100, ...
                         'VariableNames', {'node', 'action', 'energy'});
        end

        %% Perform the next action on the schedule
        function run(obj)
            obj.msg = "";
            % move to target
            obj.idle = obj.node == obj.schedule.node(1);
            obj.move(obj.schedule.node(1));
            obj.time = obj.schedule.Time(1);
            for r = 1:length(obj.mission.robots)
                robot = obj.mission.robots(r);
                if robot.state == "idle"
                    robot.node = obj.node;
                    robot.update_maps();
                end
            end
            obj.schedule(1,:) = [];
            % history
            obj.history(obj.time, :) = {obj.node};
        end

        %% Path planner
        function path_planner(obj)
            if obj.policy.optimizer == "static"
                obj.schedule = timetable(seconds(inf), obj.node, 'VariableNames', {'node'});
            else
                t0 = tic;
                obj.opt_results = feval(obj.policy.optimizer, obj);
                obj.pp_outputs(obj.time) = obj.opt_results;
                obj.msg = sprintf('%-10s | %-30s | %.4f', obj.id, obj.policy.optimizer, toc(t0));
            end
        end

        %% Move the robot 
        function move(obj, target)
            if obj.node == target
                return
            end
            edge = findedge(obj.world.environment, obj.node, target);
            if edge
                c = obj.traversability(obj.world.environment.Edges.EndNodes(edge,:));
                if c == 0 
                    error('The edge is too steep, check the path planner');
                end
            else
                error('Given target is not a neighbor of the robot node');
            end
            obj.node = target;
        end

        %% Traversability of the environment
        function c = traversability(obj, edges)
            steps = abs(obj.world.environment.Nodes.terrain(obj.world.environment.findnode(edges(:,1))) - ...
                        obj.world.environment.Nodes.terrain(obj.world.environment.findnode(edges(:,2))));
            steps(isnan(steps)) = 0;
            c = steps <= obj.max_step;
        end

        %% Initialize GUI handles
        function handles = init_gui(obj, parent_handle)
            hold(parent_handle, 'on');
            % position handle
            handles.position = plot(parent_handle, ...
                                    obj.world.X(str2double(obj.node)),...
                                    obj.world.Y(str2double(obj.node)), ...
                                    'pentagram', ...
                                    'MarkerEdgeColor', 'black', ...
                                    'MarkerFaceColor', obj.color, ...
                                    'MarkerSize', 10);
            % path handles
            handles.path = plot(parent_handle,...
                                obj.world.X(str2double(obj.node)),...
                                obj.world.Y(str2double(obj.node)),...
                                'Color', 'white', ...
                                'LineWidth', 1);
            % map handles
            handles.map = imagesc(parent_handle, ...
                                 [obj.world.X(1,1) obj.world.X(1,end)], ...
                                 [obj.world.Y(1,1) obj.world.Y(end,1)], ...
                                 zeros([obj.world.size(), 3]));
            hold(parent_handle, 'off');
        end

        %% Update GUI handles
        function update_gui(obj, handles)
            % update position
            set(handles.position, ...
                'XData', obj.world.X(str2double(obj.node)), ...
                'YData', obj.world.Y(str2double(obj.node)));
            % update paths
            set(handles.path, ...
                'XData', obj.world.X(str2double(obj.schedule.node)), ...
                'YData', obj.world.Y(str2double(obj.schedule.node)));
            % update maps
            if isfield(obj.opt_results, 'candidates')
                c = 0.4 * ones(obj.world.size());
                c(str2double(obj.opt_results.candidates)) = 0;
                set(handles.map, 'AlphaData', c);
            end

        end
    end
end