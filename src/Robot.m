classdef Robot < handle
    %ROBOT Generic robot implementation
    
    properties
        % params fields
        id string;
        color string;
        q_init cell;
        charge_per_s double; 
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
        state string = "running"';

        world World;
        mission Mission;

        history timetable;
        pp_outputs;
        return_schedule timetable;
        cache table; 
        msg string = "";
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

            obj.world = world;
            obj.mission = mission;
            
            obj.id = params.id;
            obj.color = params.color; 
            obj.charge_per_s = params.charge_per_s;
            obj.max_step = params.max_step;
            obj.speed = params.speed;
            obj.height = params.height;
            obj.crit_energy = params.crit_energy;
            obj.energy_per_m = params.energy_per_m;
            obj.policy = params.policy;
            obj.pp_outputs = dictionary();
            
            % place the robot 
            obj.q_init = mission.q_init;
            obj.node = world.get_id(obj.q_init{1}, obj.q_init{2});
            obj.time = seconds(0);
            obj.time.Format = 'hh:mm:ss';
            obj.schedule = timetable(duration.empty(0,1), ...
                string.empty, string.empty, [], ...
                'VariableNames', {'node', 'action', 'energy'});
            obj.return_schedule = timetable(duration.empty(0,1), ...
                string.empty, string.empty, [], ...
                'VariableNames', {'node', 'action', 'energy'});
            
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
            obj.history = timetable(obj.time, obj.node, "none", 0, 0, 0, 0,...
                'VariableNames', {'node', ...
                                  'action', ...
                                  'distance', ...
                                  'mapped_area', ...
                                  'detected_victims', ...
                                  'energy'});
        end

        %% Perform the next action on the schedule
        function results = run(obj)
            % clear previous measurements if applicable
            for i = 1:length(obj.capabilities)
                obj.(obj.capabilities(i)).measurements = table();
            end
            obj.msg = "";
            % preliminaries
            obj.time = obj.schedule.Time(1);
            obj.energy = obj.schedule.energy(1);
            results.tt = obj.schedule(1,:);
            obj.schedule(1,:) = [];

            if obj.state == "idle"
                obj.update_maps();
                obj.energy = 100;
                results.pp = obj.path_planner();
                if results.pp.charge_flag
                    % no tasks are viable
                    candidate_times = obj.mission.charger.schedule.Time(1) + seconds(1);
                    filtered_actions = ["none" "charge" "charge_done"];
                    for r = 1:length(obj.mission.robots)
                        if obj.mission.robots(r).state == "running" && ...
                           ~ismember(obj.mission.robots(r).schedule.action(end), filtered_actions)
                            candidate_times(end+1) = obj.mission.robots(r).schedule.Time(end);
                        end
                    end
                    obj.schedule = timetable(min(candidate_times), ...
                        obj.node, "none", 100, ...
                        'VariableNames', {'node', 'action', 'energy'});
                else
                    % a new plan can be made
                    obj.generate_schedule([results.pp.tasks.node], results.pp.actions, obj.time);
                    obj.state = "running";
                end

            elseif obj.state == "running"
                % move to target
                obj.move(results.tt.node(1));
                % parse the action
                action = results.tt.action(1).split('_');
                if action(1) == "map"
                    obj.mapper.measure(str2double(action(2)));
                    obj.update_maps();
                    obj.control_step = obj.control_step + 1;
                    results.tasks = obj.mission.manage_tasks(obj, results.tt);
                elseif action(1) == "search"
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
                    results.tasks = obj.mission.manage_tasks(obj, results.tt);
                elseif action(1) == "charge"
                    obj.mission.charger.charge(obj);
                    obj.state = "idle";
                end
                
                % check if a new plan needs to be made
                if isempty(obj.schedule) 
                    results.pp = obj.path_planner();
                    % if charge_flag is raised, use the return path
                    if results.pp.charge_flag
                        obj.schedule = obj.return_schedule;
                        obj.return_schedule(:,:) = [];
                    else
                        obj.generate_schedule([results.pp.tasks.node], results.pp.actions, obj.time);
                        % check for conflicts
                        [conflicts, conflict_schedules] = detect_conflicts(obj.mission);
                        if ~isempty(conflicts)
                            t0 = tic;
                            results.coop = cooperation(obj.mission, conflict_schedules, conflicts);
                            obj.msg = obj.msg +  sprintf('\n%-10s | %-30s | %.4f', obj.id, 'cooperation', toc(t0));
                        end
                    end
                end
            end
            obj.log_history(results);
        end

        %% Log the lates robot state
        function log_history(obj, results)
            h_node = obj.node;
            h_action = results.tt.action(1);
            [~, d] = obj.world.environment.shortestpath(h_node, obj.history.node(end));
            h_distance = obj.history.distance(end) + d;
            h_mapped_area = obj.history.mapped_area(end);
            if ismember("mapper", obj.capabilities) && ~isempty(obj.mapper.measurements)
                h_mapped_area = h_mapped_area + sum(obj.mapper.measurements.is_new);
            end
            new_victims = sum([obj.world.victims.t_detected] == obj.time);
            if h_action == "none"
                new_victims = 0;
            end
            h_detected_victims = obj.history.detected_victims(end) + new_victims;
            h_energy = obj.energy;
            
            obj.history(end+1,:) = {h_node, ...
                                    h_action, ...
                                    h_distance, ...
                                    h_mapped_area, ...
                                    h_detected_victims, ...
                                    h_energy};   
            obj.history.Time(end) = obj.time;
            % save path planner results if applicable
            if isfield(results, 'pp')
                obj.pp_outputs(obj.time) = results.pp;
            end
        end

        %% Path planner
        function opt_results = path_planner(obj)
            % update current map
            obj.update_maps();
            % find candidate tasks and preprocess them
            pp = preprocessing(obj);

            % employ task allocation strategy
            optimizer_fcn = obj.policy.optimizer + "_task_allocator";
            t0 = tic;
            opt_results = feval(optimizer_fcn, obj, pp);
            obj.msg = sprintf('%-10s | %-30s | %.4f', obj.id, optimizer_fcn, toc(t0));

            % results
            obj.cache = opt_results.cache; 
            opt_results.robot = obj;
            obj.control_step = 0;
        end


        %% Generate schedule with given node - action pairs
        function generate_schedule(obj, nodes, actions, start_time)
            nodes = [obj.node; nodes(:)];
            last_time = start_time;
            last_energy = obj.energy;
            for i = 1:length(actions)
                % construct the path
                [path, ~, edges] = obj.map.shortestpath(nodes(i), nodes(i+1));
                for p = 2:length(path)-1
                    d = obj.map.Edges.Weight(edges(p-1));
                    last_time = last_time + seconds(d / obj.speed);
                    last_energy = last_energy - d * obj.energy_per_m;
                    obj.schedule(last_time, :) = {path(p), "none", last_energy};
                end
                % task action
                action = actions(i).split('_');
                if action(1) == "map"
                    last_time = last_time + obj.mapper.t_s;
                    last_energy = last_energy - obj.mapper.d_energy;
                elseif action(1) == "search"
                    last_time = last_time + obj.detector.t_s;
                    last_energy = last_energy - obj.detector.d_energy;
                end
                obj.schedule(last_time, :) = {nodes(i+1), actions(i), last_energy};
            end
            % generate the return schedule from the last action
            charger_nodes = [obj.mission.charger.node;
                             obj.mission.charger.schedule.node];
            charger_times = [obj.mission.charger.schedule.Time;
                             seconds(inf)];
            obj.return_schedule = generate_return_path(obj, ...
                                                       nodes(end), ...
                                                       last_time, ...
                                                       last_energy, ...
                                                       charger_nodes, ...
                                                       charger_times, ...
                                                       true);
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

        %% Update the global and local maps with the current measurements
        % Assuming no measurement errors on 'terrain'
        function update_maps(obj)
            if ismember("mapper", obj.capabilities) && ~isempty(obj.mapper.measurements)
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

