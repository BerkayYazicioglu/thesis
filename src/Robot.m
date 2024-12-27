classdef Robot < handle
    %ROBOT Generic robot implementation
    
    properties
        % params fields
        id string;
        color string;
        q_init cell;
        charge_s duration; 
        max_step double {mustBeReal, mustBeNonnegative}; % m 
        speed double {mustBeReal, mustBeNonnegative}; % m/s
        height double {mustBeNonnegative, mustBeReal}; % m
        crit_energy double {mustBeNonnegative}; % percentage 
        energy_per_m double {mustBeNonnegative}; % percentage
        policy; % struct for policy settings

        node string;
        time duration;
        mapper Mapper;
        detector Detector; 
        map graph = graph;
        schedule timetable;
        energy double {mustBeNonnegative} = 100; % percentage energy
        capabilities (1,:) string {mustBeVector} = string.empty;
        control_step double = 0;
        idle logical = false;

        world World;
        mission Mission;

        history timetable;
        cache table; 
    end
    
    methods
        %% Constructor
        function obj = Robot(params, world, mission)
            %ROBOT params (struct) ->
            %   id          : robot id
            %   color       : plotting color
            %   charge_s    : (duration) charging time
            %   max_step    : maximum step increase the robot can climb (m)
            %   speed       : travel speed m/s
            %   height      : height of the robot above ground (m)
            %   crit_energy : (%) critical power percentage
            %   energy_per_m: (%) energy expenditure percentage per meter
            %   PI_model    : path of the PI model
            %   policy      : policy settings
            %   capabilities: capability settings
            %   q_init      : initial grid location of robot [x y]

            obj.world = world;
            obj.mission = mission;
            
            obj.id = params.id;
            obj.color = params.color; 
            obj.charge_s = duration(params.charge_s, 'InputFormat', 'mm:ss');
            obj.charge_s.Format = 's';
            obj.max_step = params.max_step;
            obj.speed = params.speed;
            obj.height = params.height;
            obj.crit_energy = params.crit_energy;
            obj.energy_per_m = params.energy_per_m;
            obj.policy = params.policy;
            
            % place the robot 
            obj.q_init = params.q_init;
            obj.node = world.get_id(obj.q_init{1}, obj.q_init{2});
            obj.time = seconds(0);
            obj.time.Format = 'hh:mm:ss';
            obj.schedule = timetable(duration.empty(0,1), string.empty, string.empty, ...
                'VariableNames', {'node', 'action'});
            
            % capabilities
            obj.capabilities = [cellfun(@(x) string(x), fields(params.capabilities))];
            for i = 1:length(obj.capabilities)
                capability = obj.capabilities(i);
                if capability == "mapper"
                    obj.mapper = Mapper(params.capabilities.mapper, obj);
                elseif capability == "detector"
                    obj.detector = Detector(params.capabilities.detector, obj);
                end
            end

            % history
            obj.history = timetable(obj.time, obj.node, "none", 0, 0, 0, 0 ,...
                'VariableNames', {'node', ...
                                  'action', ...
                                  'distance', ...
                                  'mapped_area', ...
                                  'detected_victims', ...
                                  'energy'});
        end

        %% Perform the next action on the schedule
        function tt = run(obj)
            % clear previous measurements if applicable
            for i = 1:length(obj.capabilities)
                obj.(obj.capabilities(i)).measurements = table();
            end
            % move to target
            obj.move(obj.schedule.node(1));
            % parse the action
            action = obj.schedule.action(1).split('_');
            if action(1) == "map"
                obj.energy = obj.energy - obj.mapper.d_energy;
                obj.time = obj.time + obj.mapper.t_s;
                obj.mapper.measure(str2double(action(2)));
                obj.update_maps();
                obj.control_step = obj.control_step + 1;
            elseif action(1) == "search"
                obj.energy = obj.energy - obj.mapper.d_energy;
                obj.time = obj.time + obj.detector.t_s;
                obj.detector.measure(str2double(action(2)));
                % update detected victims
                found_idx = find(obj.detector.measurements.victim);
                victim_nodes = obj.detector.measurements.nodes(found_idx);
                for i = 1:length(obj.world.victims)
                    if ismember(obj.world.victims(i).node, victim_nodes)
                        obj.world.victims(i).detect(obj);
                    end
                end
                obj.control_step = obj.control_step + 1;
            elseif action(1) == "charge"
                obj.mission.charger.charge(obj);
            end
            % log history
            obj.log_history();
            % update schedule
            tt = obj.schedule(1,:);
            obj.schedule(1,:) = [];
        end

        %% Log the lates robot state
        function log_history(obj)
            h_node = obj.node;
            h_action = obj.schedule.action(1);
            [~, d] = obj.world.environment.shortestpath(h_node, obj.history.node(end));
            h_distance = obj.history.distance(end) + d;
            h_mapped_area = obj.history.mapped_area(end);
            if ~isempty(obj.mapper.measurements)
                h_mapped_area = h_mapped_area + sum(obj.mapper.measurements.is_new);
            end
            new_victims = sum([obj.world.victims.t_detected] == obj.time);
            h_detected_victims = obj.history.detected_victims(end) + new_victims;
            h_energy = obj.energy;
            
            obj.history(obj.time,:) = {h_node, ...
                                       h_action, ...
                                       h_distance, ...
                                       h_mapped_area, ...
                                       h_detected_victims, ...
                                       h_energy};   
            end

        %% Path planner
        function opt_results = path_planner(obj)
            % update current map
            obj.update_maps();
            % find candidate tasks and preprocess them
            pp = preprocessing(obj);

            % employ task allocation strategy
            optimizer_fcn = obj.policy.optimizer + "_task_allocator";
            opt_results = feval(optimizer_fcn, obj, pp);

            % generate the schedule
            last_time = obj.generate_schedule([opt_results.tasks.node], opt_results.actions);
            % if charge_flag is raised, construct the return path
            if opt_results.charge_flag
                return_schedule = generate_return_path(obj);
                return_schedule.Time = return_schedule.Time + last_time;
                obj.schedule = [obj.schedule; return_schedule];
            end
            obj.cache = opt_results.cache; 
            opt_results.robot = obj;
            obj.control_step = 0;
            obj.idle = obj.energy == 100 && opt_results.charge_flag;
        end


        %% Generate schedule with given node - action pairs
        function last_time = generate_schedule(obj, nodes, actions)
            nodes = [obj.node; nodes(:)];
            last_time = obj.time;
            for i = 1:length(actions)
                % construct the path
                [path, ~, edges] = obj.map.shortestpath(nodes(i), nodes(i+1));
                for p = 2:length(path)-1
                    last_time = last_time + seconds(obj.map.Edges.Weight(edges(p-1))/obj.speed);
                    obj.schedule(last_time, :) = {path(p), "none"};
                end
                % task action
                action = actions(i).split('_');
                if action(1) == "map"
                    last_time = last_time + obj.mapper.t_s;
                elseif action(1) == "search"
                    last_time = last_time + obj.detector.t_s;
                end
                obj.schedule(last_time, :) = {nodes(i+1), actions(i)};
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
                distance = obj.world.environment.Edges(edge,:).Weight;
            else
                error('Given target is not a neighbor of the robot node');
            end
            obj.energy = obj.energy - distance * obj.energy_per_m;
            obj.time = obj.time + seconds(distance / obj.speed);
            obj.node = target;
        end

        %% Traversability of the environment
        function c = traversability(obj, edges)
            steps = abs(obj.world.environment.Nodes.terrain(obj.world.environment.findnode(edges(:,1))) - ...
                        obj.world.environment.Nodes.terrain(obj.world.environment.findnode(edges(:,2))));
            steps(isnan(steps)) = 0;
            c = steps <= obj.max_step;
        end

        %% Update the global and local maps with the current measurements
        % Assuming no measurement errors on 'terrain'
        function update_maps(obj)
            if ~isempty(obj.mapper.measurements)
                % add measurements to the global map
                sub = subgraph(obj.world.environment, ...
                               obj.mapper.measurements.nodes); 
                sub.Nodes.visible = obj.mapper.measurements.visible;
                % get the new nodes with terrain information
                if isempty(obj.mission.map.Nodes)
                    new_nodes = sub.Nodes(:, ["Name", obj.mapper.features{:}, "visible"]);
                else
                    new_node_idx = ~ismember(sub.Nodes.Name, obj.mission.map.Nodes.Name);
                    new_nodes = sub.Nodes(new_node_idx, ["Name", obj.mapper.features{:}, "visible"]);
                    % update old nodes
                    old_nodes = obj.mission.map.findnode(sub.Nodes.Name(~new_node_idx));
                    obj.mission.map.Nodes.visible(old_nodes) = ...
                        obj.mission.map.Nodes.visible(old_nodes) | sub.Nodes.visible(~new_node_idx);
                end
                % add the subgraph to the map
                obj.mission.map = obj.mission.map.addnode(new_nodes);
                obj.mission.map = obj.mission.map.addedge(sub.Edges);
                % remove repeated entries
                obj.mission.map = obj.mission.map.simplify();
            end
        
            % update local map from the global map
            c = find(~obj.traversability(obj.mission.map.Edges.EndNodes));
            obj.map = obj.mission.map.rmedge(obj.mission.map.Edges.EndNodes(c,1), ...
                                             obj.mission.map.Edges.EndNodes(c,2));
            obj.map = obj.map.rmnode(obj.map.Nodes.Name(~obj.map.Nodes.visible));
        end

        %% Initialize GUI handles
        function handles = init_gui(obj, parent_handle)
            hold(parent_handle, 'on');
            % map handles
            handles.map = imagesc(parent_handle, ...
                                 [obj.world.X(1,1) obj.world.X(1,end)], ...
                                 [obj.world.Y(1,1) obj.world.Y(end,1)], ...
                                 zeros([obj.world.size(), 3]));
            % robot position handle
            handles.position = plot(parent_handle, ...
                                    obj.world.X(str2double(obj.node)),...
                                    obj.world.Y(str2double(obj.node)), ...
                                    'o', 'MarkerFaceColor', obj.color, ...
                                    'MarkerEdgeColor', 'red', ...
                                    'MarkerSize', 7);
            % path handles
            handles.path = plot(parent_handle,...
                                obj.world.X(str2double(obj.node)),...
                                obj.world.Y(str2double(obj.node)),...
                                'Color', 'red', ...
                                'LineWidth', 1.5);
            % capability handles
            for i = 1:length(obj.capabilities)
                capability = obj.capabilities(i);
                handles.capabilities.(capability) = obj.(capability).init_gui(parent_handle);
            end

            hold(parent_handle, 'off');
        end

        %% Update GUI handles
        function update_gui(obj, handles)
            % update maps
            c = 0.4 * ones(obj.world.size());
            c(str2double(obj.map.Nodes.Name)) = 0;
            [bin, ~] = conncomp(obj.map);
            accessible = 0 * bin;
            robot_bin = bin(obj.map.findnode(obj.node));
            accessible = accessible | (bin == robot_bin);
            c(cellfun(@str2double, obj.map.Nodes.Name(~accessible))) = 1;
            set(handles.map, 'AlphaData', c);
            % update capabilities
            if isfield(handles.capabilities, 'mapper') 
                obj.mapper.update_gui(handles.capabilities.mapper);
            end
            if isfield(handles.capabilities, 'detector') 
                obj.detector.update_gui(handles.capabilities.detector);
            end
            % update position
            set(handles.position, ...
                'XData', obj.world.X(str2double(obj.node)), ...
                'YData', obj.world.Y(str2double(obj.node)));
            % update paths
            set(handles.path, ...
                'XData', obj.world.X(str2double(obj.schedule.node)), ...
                'YData', obj.world.Y(str2double(obj.schedule.node)));
        end
    end
end

