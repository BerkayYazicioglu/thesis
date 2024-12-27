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
        R_k timetable; % docked robots until corresponding time

        world World;
        mission Mission;
        history timetable;
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
            %   q_init      : initial grid location of robot [x y]

            obj.world = world;
            obj.mission = mission;
            
            obj.id = params.id;
            obj.color = params.color; 
            obj.max_step = params.max_step;
            obj.speed = params.speed;
            obj.policy = params.policy;
            
            % place the robot 
            obj.q_init = params.q_init;
            obj.node = world.get_id(obj.q_init{1}, obj.q_init{2});
            obj.time = seconds(0); 
            obj.time.Format = 'hh:mm:ss';
            obj.schedule = timetable(obj.time, obj.node, ...
                'VariableNames', {'node'});
            obj.R_k = timetable(duration.empty(0,1), string.empty, ...
                'VariableNames', {'robot_id'});

            % history
            obj.history = timetable(obj.time, obj.node,...
                'VariableNames', {'node'});
        end

        %% Charge a robot
        function charge(obj, robot)
            if obj.node ~= robot.node
                error("Robot can not be charged, needs to be on the same node");
            end
            robot.energy = 100;
            robot.time = robot.time + robot.charge_s;
            obj.R_k(end+1,:) = {robot.id};
            obj.R_k.Time(end) = robot.time;
        end

        %% Perform the next action on the schedule
        function run(obj)
            % move to target
            obj.move(obj.schedule.node(1));
            obj.time = obj.schedule.Time(1);
            % update Rk
            delete_row_idx = [];
            for i = 1:height(obj.R_k)
                robot = obj.mission.robots([obj.mission.robots.id] == obj.R_k.robot_id(i));
                if robot.time > obj.R_k.Time(i)
                    % robot moved away
                    delete_row_idx(end+1) = i; 
                else
                    % robot is still docked
                    robot.node = obj.node;
                end
            end
            obj.R_k(delete_row_idx,:) = [];
            obj.schedule(1,:) = [];
            % history
            obj.history(obj.time, :) = {obj.node};
        end

        %% Path planner
        function path_planner(obj)
            if obj.policy.optimizer == "static"
                obj.schedule = timetable(seconds(inf), obj.node, 'VariableNames', {'node'});
            elseif obj.policy.optimizer == "dynamic"
                % remove the infeasable edges
                c = find(~obj.traversability(obj.mission.map.Edges.EndNodes));
                map = obj.mission.map.rmedge(obj.mission.map.Edges.EndNodes(c,1), ...
                                             obj.mission.map.Edges.EndNodes(c,2));
                map = map.rmnode(map.Nodes.Name(~map.Nodes.visible));
                % get only the accessible nodes
                [bin, ~] = conncomp(map);
                candidates = map.Nodes.Name(bin == bin(map.findnode(obj.node)));
                % get all candidates that are within task proximity range
                task_nodes = unique([obj.mission.tasks.node]);
                valid_area = [arrayfun(@(x) obj.mission.map.nearest(x, ...
                    obj.policy.task_proximity, ...
                    'Method', 'unweighted')', task_nodes, ...
                    'UniformOutput', false)];
                valid_area = unique([valid_area{:}]);
                valid_area = setdiff(valid_area, task_nodes);
                valid_area = [valid_area obj.node];
                % filter candidates that match the valid area
                candidates = candidates(ismember(candidates, valid_area));

                % find the unweighted center of mass of tasks
                locs = obj.world.get_coordinates(task_nodes);
                com = [mean(locs(:,1)) mean(locs(:,2))];
                % find the candidate closest to the goal
                candidate_locs = obj.world.get_coordinates(candidates);
                distances = vecnorm((com - candidate_locs)');
                [~, idx] = min(distances);
                candidate_goal = candidates(idx);
                [p, ~, edges] = map.shortestpath(obj.node, candidate_goal);

                % make sure to not plan before the robot predictions
                robot_max_t = seconds(zeros(1,length(obj.mission.robots)));
                for r = 1:length(obj.mission.robots)
                    % robot is on the way to dock
                    if ~isempty(obj.mission.robots(r).schedule)
                        robot_max_t(r) = obj.mission.robots(r).schedule.Time(end);
                    else
                        robot_max_t(r) = obj.mission.robots(r).time;
                    end
                end
                t_start = max(obj.time, max(robot_max_t)) + seconds(1);
                % generate the schedule
                if length(p) > 1
                    dt = [arrayfun(@(x) obj.world.environment.Edges(x,:).Weight / obj.speed, edges)];
                    t = seconds([arrayfun(@(x) sum(dt(1:x)), 1:length(dt))]) + t_start;
                    p(1) = [];
                else
                    t = t_start;
                end
                max_length = min(obj.policy.control_horizon, length(p));
                t = t(1:max_length);
                p = p(1:max_length);
                for i = 1:length(t)
                    obj.schedule(t(i),:) = {p(i)}; 
                end
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
            hold(parent_handle, 'off');
        end

        %% Update GUI handles
        function update_gui(obj, handles)
            % update position
            set(handles.position, ...
                'XData', obj.world.X(str2double(obj.node)), ...
                'YData', obj.world.Y(str2double(obj.node)));
            % update paths
        end
    end
end