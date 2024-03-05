%% General class for robots

classdef Robot < handle
    
    properties  
        world World;

        node char;
        tasks TaskManager;
        time duration;
        max_step double {mustBeReal, mustBeNonnegative}; % m 
        speed double {mustBeReal, mustBeNonnegative}; % m/s
        height double {mustBeNonnegative, mustBeReal}; % m
        crit_energy double {mustBeNonnegative}; % percentage 
        energy_per_m double {mustBeNonnegative}; % percentage

        visible_sensor Sensor;
        remote_sensor Sensor;
        measurements table;
        PI_model mamfis;
        energy double {mustBeNonnegative} = 100; % percentage energy

        map graph = graph;
        map_features cell;

        % ======= WIP ======== 
        policy 
        % ====================

        id string
        color string
        handles
    end

    methods
        %% Constructor
        function self = Robot(params, s_params, world, tasks)
            % parameters
            % id            : robot id
            % color         : plotting color
            % visible_range : (m) visible sensor range
            % remote_range  : (m) remote sensor range
            % crit_energy   : (%) critical power percentage
            % energy_per_m  : (%) energy expenditure percentage per meter
            % speed         : travel speed m/s
            % max_step      : maximum step increase the robot can climb (m)
            % height        : height of the robot above ground (m)
            % PI_model      : path of the PI model
            %
            % s_params      : struct containing sensor settings
            self.world = world;
            self.tasks = tasks;

            self.id = params.id;
            self.color = params.color;

            self.time = seconds(0);
            self.max_step = params.max_step;
            self.speed = params.speed;
            if isstring(params.height)
                self.height = Inf;
            else
                self.height = params.height;
            end
            self.crit_energy = params.crit_energy;
            self.energy_per_m = params.energy_per_m;
            self.PI_model = readfis(params.PI_model);
            self.map_features = params.map_features;

            % visible sensor
            vs_params = s_params.visible_sensor;
            vs_params.max_range = params.visible_range;
            self.visible_sensor = Sensor(vs_params, self.world);

            % remote sensor
            rs_params = s_params.remote_sensor;
            rs_params.max_range = params.remote_range;
            self.remote_sensor = Sensor(rs_params, self.world);   

            % policy
            self.policy.frontier = {};
        end

        %% Place robot on the closest point to the given coordinates
        function place(self, pos)
            dif_x = abs(self.world.X(1,:) - pos(1));
            dif_y = abs(self.world.Y(:,1) - pos(2));

            [~, i_x] = min(dif_x);
            [~, i_y] = min(dif_y);

            self.node = num2str(sub2ind(self.world.grid_dim, i_y, i_x));
        end

        %% Perform a given action
        function [de, dt] = perform(self, action, simplify)
            arguments
                self Robot
                action string
                simplify logical = false;
            end
            de = 0;
            dt = 0;

            % explore tasks
            if action == "explore"
                [de, dt] = self.visible_sensor.measure(self.height, ...
                                                       str2double(self.node));
                % perform applicable tasks
                for i = 1:length(self.visible_sensor.measurements.nodes)
                    task_node = num2str(self.visible_sensor.measurements.nodes(i));
                    if self.tasks.items.isKey(task_node)
                        % there is a task at the node, try to perform
                        if self.tasks.items(task_node).type == "explore"
                            outcome = self.tasks.items(task_node).perform_task(self, simplify);
                            if outcome
                                % if the task could be done, remove it
                                self.tasks.kill(task_node); 
                                % exploration -> search depending on the PI
                                PI = evalfis(self.PI_model, ...
                                             [self.visible_sensor.measurements.destruction(i) ...
                                              self.visible_sensor.measurements.population(i)]);
                                self.tasks.spawn("search", PI, ...
                                                 self.time + dt, ...
                                                 task_node, ...
                                                 PI);
                            end
                        end
                    end
                end
            % search tasks
            elseif action == "search"
                [de, dt] = self.remote_sensor.measure(self.height, ...
                                                      str2double(self.node));
                % perform applicable tasks
                for i = 1:length(self.remote_sensor.measurements.nodes)
                    task_node = num2str(self.remote_sensor.measurements.nodes(i));
                    if self.tasks.items.isKey(task_node)
                        % there is a task at the node, try to perform
                        if self.tasks.items(task_node).type == "search"
                            outcome = self.tasks.items(task_node).perform_task(self, simplify);
                            if outcome
                                % if the task could be done, remove it
                                self.tasks.kill(task_node); 
                            end
                        end
                    end
                end
                
            end
        end

        %% Update the global and local maps with the current measurements
        % Assuming no measurement errors on 'terrain'
        function map = update_maps(self, map)
            % add measurements to the global map
            sub = subgraph(self.world.environment, ...
                           self.visible_sensor.measurements.nodes);  
            % get the nodes on graph and their corresponding uncertainty
            idx = sub.findnode(arrayfun(@num2str, ...
                                        self.visible_sensor.measurements.nodes, ...
                                        'UniformOutput', ...
                                        false));
            sub.Nodes.uncertainty = self.visible_sensor.measurements.uncertainty(idx);
            % get the new nodes with terrain information
            if isempty(map.Nodes)
                new_nodes = sub.Nodes(:, ["Name", self.map_features, "uncertainty"]);
            else
                new_nodes = sub.Nodes(~ismember(sub.Nodes.Name, map.Nodes.Name), ...
                                      ["Name", self.map_features, "uncertainty"]);
            end
            % add the subgraph to the map
            map = map.addnode(new_nodes);
            map = map.addedge(sub.Edges);
            % remove repeated entries
            map = map.simplify();
            % identify nodes on map where the uncertainty is larger 
            node_idx = map.findnode(sub.Nodes.Name);
            uncertainty = map.Nodes.uncertainty(node_idx);
            uncertainty_idx = uncertainty > sub.Nodes.uncertainty;
            uncertainty(uncertainty_idx) = sub.Nodes.uncertainty(uncertainty_idx);
            map.Nodes.uncertainty(node_idx) = uncertainty;
        
            % update local map from the global map
            self.map = map;
            % remove the infeasable edges
            c = find(~self.traversability(map.Edges.EndNodes));
            self.map = self.map.rmedge(map.Edges.EndNodes(c,1), ...
                                       map.Edges.EndNodes(c,2));
        end

        %% Traversability of the environment
        function c = traversability(self, edges)
            steps = abs(self.map.Nodes.terrain(self.map.findnode(edges(:,1))) - ...
                        self.map.Nodes.terrain(self.map.findnode(edges(:,2))));
            %steps(isnan(steps)) = 0;
            c = steps <= self.max_step;
        end

        %% Make a decision to take an action
        function map = run(self, map)
            path = self.path_planner();
            self.move(path{1});

            action = "explore";
            % perform action
            [de, dt] = self.perform(action, true);
            self.time = self.time + dt;
            e = self.energy - de;
            if e < 0  e = 0; end
            self.energy = e;

            % update maps
            if action == "explore"
                map = self.update_maps(map);
            end
        end
        

        %% Move the robot towards a target node 
        function move(self, target)
            edge = findedge(self.world.environment, self.node, target);
            if edge
                c = self.traversability(self.world.environment.Edges.EndNodes(edge,:));
                if c == 0 
                    error('The edge is too steep, check the path planner');
                end
                distance = self.world.environment.Edges(edge,:).Weight;
            else
                return
            end
            dt = distance / self.speed;
            e = self.energy - distance * self.energy_per_m;
            if e < 0  e = 0; end
            self.energy = e;
            self.time = self.time + seconds(dt);
            self.node = target;
        end

        %% Plan a path
        function P = path_planner(self)
            % find frontier points
            self.policy.frontier = find_candidate_points(self);
            paths = cell(1, length(self.policy.frontier));
            % create paths to frontiers
            for i = 1:length(self.policy.frontier)
                path = self.map.shortestpath(self.node, ...
                                             self.policy.frontier{i});
                path(1) = [];   
                paths{i} = path;
            end
            % allocate optimal tasks to the paths

            actions = cell(1, length(paths));
            utilites = zeros(1, length(paths));
            for i = 1:length(paths)
                if ~isempty(paths{i})
                    [actions{i}, utilities(i)] = ga_task_allocator(self, paths{i});
                end
            end
            [~, idx] = max(utilities,[],"all");


            P = paths{idx};
        end
       
        %% Plotter
        function plot(self, gui)
            if nargin > 1
                self.visible_sensor.plot(gui);
                self.remote_sensor.plot(gui);
                self.handles.pos_world = plot(gui.world, ...
                                             self.world.X(str2double(self.node)), ...
                                             self.world.Y(str2double(self.node)), ...
                                             '.', 'Color', self.color, ...
                                             'MarkerSize', 20);
                % policy
                self.policy.handles.frontier = scatter(gui.world, ...
                                                        [], ...
                                                        [], ...
                                                        20, ...
                                                        'Marker', 'o', ...
                                                        'MarkerFaceColor', 'red', ...
                                                        'MarkerFaceAlpha', 1);
            end
            self.visible_sensor.plot();
            self.remote_sensor.plot();
            set(self.handles.pos_world, ...
                'XData', self.world.X(str2double(self.node)), ...
                'YData', self.world.Y(str2double(self.node)) ...
            );
            % policy
            p = self.world.environment.findnode(self.policy.frontier);
            if isempty(p)
                set(self.policy.handles.frontier, ...
                    'XData', [], ...
                    'YData', []);
            else
                set(self.policy.handles.frontier, ...
                    'XData', self.world.X(p), ...
                    'YData', self.world.Y(p));
            end
        end
    end
end


