%% General class for robots

classdef Robot < handle
    
    properties  
        world World;
        directions double = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4];

        node char;
        heading double;
        schedule timetable;
        max_step double {mustBeReal, mustBeNonnegative}; % m 
        speed double {mustBeReal, mustBeNonnegative}; % m/s
        height double {mustBeNonnegative, mustBeReal}; % m
        crit_energy double {mustBeNonnegative}; % percentage 
        energy_per_m double {mustBeNonnegative}; % percentage

        visible_sensor Sensor;
        remote_sensor Sensor;
        PI_model mamfis;
        energy double {mustBeNonnegative} = 100; % percentage energy

        map graph = graph;
        map_features cell;

        % ======= WIP ======== 
        policy Policy
        % ====================

        id string
        color string
        params
    end

    methods
        %% Constructor
        function self = Robot(params, world, tasks)
            % parameters
            % id            : robot id
            % heading       : heading index of the directions array
            % color         : plotting color
            % crit_energy   : (%) critical power percentage
            % energy_per_m  : (%) energy expenditure percentage per meter
            % speed         : travel speed m/s
            % max_step      : maximum step increase the robot can climb (m)
            % height        : height of the robot above ground (m)
            % PI_model      : path of the PI model
            % policy        : policy settings
            % sensors       : sensor settings

            self.world = world;
            self.params = params;

            self.heading = params.heading;
            self.id = params.id;
            self.color = params.color;

            self.max_step = params.max_step;
            self.speed = params.speed;
            self.height = params.height;
            self.crit_energy = params.crit_energy;
            self.energy_per_m = params.energy_per_m;
            self.PI_model = readfis(params.PI_model);
            self.map_features = params.map_features;

            % visible sensor
            self.visible_sensor = Sensor(params.sensors.visible_sensor, ...
                                           self.world, ...
                                           self.directions);
            % remote sensor
            self.remote_sensor = Sensor(params.sensors.remote_sensor, ...
                                        self.world, ...
                                        self.directions);
            % policy
            p_params = params.policy;
            p_params.normalization = self.remote_sensor.t_s / self.visible_sensor.t_s;
            self.policy = Policy(p_params, tasks, self.world);
        end

        %% Make a decision to take an action
        function map = run(self, time, map)
            self.move(p);
            % perform action
            self.perform(action, time, true);
            % update maps
            if action == "explore"
                map = self.update_maps(map);
            end
        end

        %% Place robot on the closest point to the given coordinates
        function map = place(self, Time, pos, action, map)
            dif_x = abs(self.world.X(1,:) - pos(1));
            dif_y = abs(self.world.Y(:,1) - pos(2));

            [~, i_x] = min(dif_x);
            [~, i_y] = min(dif_y);

            self.node = num2str(sub2ind(self.world.grid_dim, i_y, i_x));

            self.perform(action, Time, true);
            self.schedule = timetable(Time, ...
                                      str2double(self.node), ...
                                      convertCharsToStrings(action), ...
                                      {{}}, ...
                'VariableNames', {'node', 'action', 'tasks'});
            % update maps
            if action == "explore"
                map = self.update_maps(map);
            end
        end

        %% Perform a given action
        function perform(self, action, time, simplify)
            arguments
                self Robot
                action string
                time duration
                simplify logical = false;
            end
            de = 0;

            % perform actions
            if action == "explore"
                [de, ~] = self.visible_sensor.measure(self.heading, ...
                                                      self.height, ...
                                                      str2double(self.node));
                self.remote_sensor.measurements = table();
            elseif action == "search"
                [de, ~] = self.remote_sensor.measure(self.heading, ...
                                                     self.height, ...
                                                     str2double(self.node));
                self.visible_sensor.measurements = table();
            elseif action == "none"
                self.remote_sensor.measurements = table();
                self.visible_sensor.measurements = table();
            end

            % manage tasks
            self.policy.tasks.run(self, action, time, simplify);

            e = self.energy - de;
            if e < 0  e = 0; end
            self.energy = e;
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
            steps(isnan(steps)) = 0;
            c = steps <= self.max_step;
        end

      
        %% Move the robot towards a target node 
        function move(self, target)
            if strcmp(self.node, target)
                return
            end
            edge = findedge(self.world.environment, self.node, target);
            if edge
                c = self.traversability(self.world.environment.Edges.EndNodes(edge,:));
                if c == 0 
                    error('The edge is too steep, check the path planner');
                end
                distance = self.world.environment.Edges(edge,:).Weight;
            else
                error('Given target is not a neighbor of the robot node');
            end
            vec = [self.world.X(str2double(target)) - self.world.X(str2double(self.node)) ...
                   self.world.Y(str2double(target)) - self.world.Y(str2double(self.node))];
            vec_angle = mod(atan2(vec(2), vec(1)), 2*pi);
            self.heading = find(round(self.directions, 1) == round(vec_angle, 1));
            if isempty(self.heading)
                error('Heading error');
            end

            e = self.energy - distance * self.energy_per_m;
            if e < 0  e = 0; end
            self.energy = e;
            self.node = target;
        end
    end
end


