%% General class for robots

classdef Robot < handle
    
    properties  
        world World;
        task_set GlobalTasks;
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

        id string
        color string
        handles



        prev = [];
    end
    
    properties (SetObservable)
        node double {mustBeNonnegative} = 1;
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
            self.task_set = tasks;

            self.id = params.id;
            self.color = params.color;

            self.time = seconds(0);
            self.max_step = params.max_step;
            self.speed = params.speed;
            self.height = params.height;
            self.crit_energy = params.crit_energy;
            self.energy_per_m = params.energy_per_m;
            self.PI_model = readfis(params.PI_model);

            % create map
            self.map_features = params.map_features;
            % function e = edge_fcn(source, target)
            %     source_vec = [self.world.X(source)
            %                   self.world.Y(source)];
            %     target_vec = [self.world.X(target)
            %                   self.world.Y(target)];
            %     e = norm(target_vec - source_vec);
            % end
            % % node function
            % function v = vertex_fcn(nodes)
            %     N = numel(nodes);
            %     table_data = cell(N, 2);
            %     table_data(:, 1) = arrayfun(@num2str, 1:N, 'UniformOutput', false);
            %     table_data(:, 2) = arrayfun(@(x) NaN, 1:N, 'UniformOutput', false);
            %     v = cell2table(table_data, ...
            %                    'VariableNames', ["Name", self.map_features]);
            % end
            % self.map = create_map(self.world.grid_dim, @edge_fcn, @vertex_fcn);

            % visible sensor
            vs_params = s_params.visible_sensor;
            vs_params.range = ceil(params.visible_range ./ world.cell_size(:));
            vs_params.max_range = params.visible_range;
            self.visible_sensor = Sensor(vs_params);

            % remote sensor
            rs_params = s_params.remote_sensor;
            rs_params.range = ceil(params.remote_range ./ world.cell_size(:));
            rs_params.max_range = params.remote_range;
            self.remote_sensor = Sensor(rs_params);   

            addlistener(self, 'node', 'PostSet', @self.measure);
        end

        %% Place robot on the closest point to the given coordinates
        function place(self, pos)
            dif_x = abs(self.world.X(1,:) - pos(1));
            dif_y = abs(self.world.Y(:,1) - pos(2));

            [~, i_x] = min(dif_x);
            [~, i_y] = min(dif_y);

            self.node = sub2ind(self.world.grid_dim, i_y, i_x);
        end

        %% Get measurements every time a new node is reached
        function measure(self, varargin)
            [v_energy, v_ts] = self.visible_sensor.measure(self.height, ...
                                                           self.node, ...
                                                           self.world);
            [r_energy, r_ts] = self.remote_sensor.measure(self.height, ...
                                                          self.node, ...
                                                          self.world);
            e = self.energy - v_energy - r_energy;
            if e < 0  e = 0; end
            self.energy = e;

            self.time = self.time + seconds(max(v_ts, r_ts));

            % merge data
            self.measurements = outerjoin(self.visible_sensor.measurements, ...
                                          self.remote_sensor.measurements, ...
                                          "Keys", {'nodes', 'coordinates'}, ...
                                          "MergeKeys", true);
            % calculate PI
            PI_input = [self.measurements.destruction ...
                        self.measurements.population];
            PI_values = evalfis(self.PI_model, PI_input);
            self.measurements.PI = PI_values;
            % update tasks
            %sensed_nodes = self.sensor.measurements.nodes;
            %self.world.environment.Nodes(sensed_nodes, :).task = repmat(Tasks.none, length(sensed_nodes), 1);
        end

        %% Update the global map with the current measurements
        % Assuming no measurement errors on 'terrain'
        function map = update_global_map(self, map)
            sub = subgraph(self.world.environment, ...
                           self.visible_sensor.measurements.nodes);   
            % get the new nodes with terrain information
            if isempty(map.Nodes)
                new_nodes = sub.Nodes(:, ["Name", self.map_features]);
                [~, idx] = ismember(cellfun(@str2num, new_nodes.Name), ...
                                    self.visible_sensor.measurements.nodes);
                new_nodes.uncertainty = self.visible_sensor.measurements.uncertainty(idx);
            else
                new_nodes = sub.Nodes(~ismember(sub.Nodes.Name, map.Nodes.Name), ...
                                      ["Name", self.map_features]);
                [~, idx] = ismember(cellfun(@str2num, new_nodes.Name), ...
                                    self.visible_sensor.measurements.nodes);
                new_nodes.uncertainty = self.visible_sensor.measurements.uncertainty(idx);
            end
            % add the subgraph to the map
            map = map.addnode(new_nodes);
            map = map.addedge(sub.Edges);
            % remove repeated entries
            map = map.simplify();
            % identify nodes on map where the uncertainty is larger 
            for i = 1:length(self.visible_sensor.measurements.nodes)
                % update the uncertainties 
                idx = strcmp(map.Nodes.Name, ...
                             num2str(self.visible_sensor.measurements.nodes(i)));
                uncertainty = map.Nodes.uncertainty(idx);
                if uncertainty > self.visible_sensor.measurements.uncertainty(i)
                    map.Nodes.uncertainty(idx) = self.visible_sensor.measurements.uncertainty(i);
                end
            end
        end

        %% Update the local map with the current measurements
        % Assuming no measurement errors on 'terrain'
        function update_local_map(self, map)
            % % get the terrains that are in the global map
            % for f = 1:length(self.map_features)
            %     feature = self.map_features{f};
            %     nodes = cellfun(@str2num, map.Nodes.Name);
            %     self.map.Nodes.(feature)(nodes) = map.Nodes.(feature); 
            % end
            % % update the edges from the current global map
            % self.map = self.map.addedge(map.Edges);
            % % simplify the map with keeping the updated values
            % self.map = self.map.simplify('last');

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
            self.move(path(1));
            % update maps
            map = self.update_global_map(map);
            self.update_local_map(map);
            % manage tasks
            new_tasks = self.task_set.spawn_tasks(map, self.time);
            for i = 1:length(new_tasks)
                self.task_set.add_task(new_tasks{i});
            end
            self.perform_tasks();
        end
        
        %% Perform applicable tasks
        function perform_tasks(self)
            % get the tasks that can be completed with the current
            % measurements
            for i = 1:length(self.measurements.nodes)
                task_node = self.measurements.nodes(i);
                if self.task_set.items.isKey(task_node)
                    % there is a task at the measured node
                    task = self.task_set.items(task_node);
                    if task.type == "explore"
                        if self.measurements.uncertainty_left(i) <= self.task_set.exp_kill_thresh
                            % PI value is good enough to complete the
                            % explore task
                            self.task_set.remove_task(task_node);   
                        end
                    end
                end
            end
        end

        %% Move the robot towards a target node 
        % mustBeAValidTarget(.,., x): 
        %   x -> True : check for steps
        %   x -> False: skip step checking
        function move(self, target)
            arguments
                self Robot
                target double {mustBeAValidTarget(target, self, 1)}
            end
            edge = findedge(self.world.environment, self.node, target);
            distance = self.world.environment.Edges(edge,:).Weight;
            dt = distance / self.speed;
            e = self.energy - distance * self.energy_per_m;
            if e < 0  e = 0; end
            self.energy = e;
            self.time = self.time + seconds(dt);
            self.node = target;
        end

        %% Plan a path
        function path = path_planner(self)
            % find all possible paths of search depth
            % tic;
            % 
            % paths = find_all_paths(self.world.environment, ...
            %                        self.node, ...
            %                        self.path_plan_steps, ...
            %                        @(g,s) uint16(neighbors(g, s)'));
            % 
            % fprintf('%s found total %.0f paths in %f seconds\n', self.id, length(paths), toc); 
            % % evaluate the paths
            % self.evaluate_path([], true);
            % tic;
            % [best_val, best_idx] = max(cellfun(@(path)self.evaluate_path(path, false), paths));
            % fprintf('%s evaluated all paths in %f seconds | e(path) = %.2f\n', self.id, toc, best_val);
            % self.path = paths{best_idx};
            

            if isempty(self.prev)
                self.prev = [self.node];
            end
            n = self.world.environment.neighbors(self.node);
            candidates = setdiff(n, self.prev);
            %next = candidates(randi(numel(candidates)));
            next = candidates(1);
            self.prev(end+1) = next;
            path = next;
        end
       
        %% Plotter
        function plot(self, gui)
            if nargin > 1
                self.visible_sensor.plot(gui);
                self.remote_sensor.plot(gui);
                self.handles.pos_world = plot(gui.world, ...
                                             self.world.X(self.node), ...
                                             self.world.Y(self.node), ...
                                             '.', 'Color', self.color, ...
                                             'MarkerSize', 20);
            end
            self.visible_sensor.plot();
            self.remote_sensor.plot();
            set(self.handles.pos_world, ...
                'XData', self.world.X(self.node), ...
                'YData', self.world.Y(self.node));
        end
    end
end


