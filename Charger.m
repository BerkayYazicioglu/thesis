%% Charger unit

classdef Charger < handle
    properties
        world World
        directions double = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4];

        node char;
        time duration;
        schedule timetable;
        heading double;
        max_step double {mustBeReal, mustBeNonnegative}; % m 
        speed double {mustBeReal, mustBeNonnegative}; % m/s
        height double {mustBeNonnegative, mustBeReal}; % m

        id string
        color string
        policy string
        control_horizon double
    end

    methods
        %% Constructor
        function self = Charger(params, world)
            % parameters
            % id            : robot id
            % heading       : heading index of the directions array
            % color         : plotting color
            % speed         : travel speed m/s
            % max_step      : maximum step increase the robot can climb (m)
            % height        : height of the robot above ground (m)
            % policy        : policy settings
            
            self.world = world;

            self.heading = params.heading;
            self.id = params.id;
            self.color = params.color;
            self.time = seconds(0);
            self.max_step = params.max_step;
            self.speed = params.speed;
            self.height = params.height;
            self.policy = params.policy.type;
            self.control_horizon = params.policy.control_horizon;
            
            % place the charger
            pos = [params.p_init{:}];
            dif_x = abs(self.world.X(1,:) - pos(1));
            dif_y = abs(self.world.Y(:,1) - pos(2));

            [~, i_x] = min(dif_x);
            [~, i_y] = min(dif_y);

            self.node = num2str(sub2ind(self.world.grid_dim, i_y, i_x));

            self.schedule = ...
                timetable(seconds(0), ...
                          str2num(self.node), ...
                          'VariableNames', {'node'});
        end

        %% Run the decision making
        function tt = run(self, time)
            arguments
                self Charger
                time duration
            end
            tt = self.schedule(self.schedule.Time <= time, :);
            for i = 1:height(tt)
                self.move(num2str(self.schedule.node(i)));
                self.time = self.schedule.Time(i);
            end
            self.schedule(1:height(tt), :) = [];
        end

        %% Path planner
        function path_planner(self, time, map, tasks)
            arguments
                self Charger
                time duration
                map graph
                tasks TaskManager
            end
            if self.policy == "static"
                self.schedule = [self.schedule; ...
                                timetable(seconds(inf), ...
                                          str2num(self.node), ...
                                          'VariableNames', {'node'})];
            end
            if self.policy == "dynamic"
                % find the unweighted center of mass of tasks
                locs = reshape([tasks.items.values.location], [2, tasks.items.numEntries()])';
                goal = [mean(locs(:,1)) mean(locs(:,2))];
                % remove the infeasable edges
                c = find(~self.traversability(map.Edges.EndNodes));
                map = map.rmedge(map.Edges.EndNodes(c,1), ...
                                 map.Edges.EndNodes(c,2));
                map = map.rmnode(find(map.Nodes.uncertainty >= 0.1));
                % get only the accessible nodes
                [bin, ~] = conncomp(map);
                candidates = map.Nodes.Name(bin == bin(map.findnode(self.node)));
                candidates([cellfun(@(x) strcmp(x, self.node), candidates)]) = [];
                % find the candidate closest to the goal
                candidate_locs = [self.world.X([cellfun(@(x) str2double(x), candidates)])'; 
                                  self.world.Y([cellfun(@(x) str2double(x), candidates)])']';
                distances = vecnorm((goal - candidate_locs)');
                [min_dist, idx] = min(distances);
                candidate_goal = candidates{idx};
                [p, ~, edges] = map.shortestpath(self.node, candidate_goal);
                % update schedule
                if height(self.schedule)
                    t_start = max(self.schedule.Time(end), time);
                else
                    t_start = time;
                end
                dt = [arrayfun(@(x) self.world.environment.Edges(x,:).Weight / self.speed, edges)];
                Time = seconds([arrayfun(@(x) sum(dt(1:x)), 1:length(dt))]) + t_start;
                p(1) = [];
                max_length = min(self.control_horizon, length(p));
                Time = Time(1:max_length);
                p = p(1:max_length);
                self.schedule = [self.schedule; ...
                                timetable(Time(:), ...
                                          [cellfun(@(x) str2num(x),p)]', ...
                                          'VariableNames', {'node'})];
            end
        end

        %% Traversability of the environment
        function c = traversability(self, edges)
            steps = abs(self.world.environment.Nodes.terrain(...
                        self.world.environment.findnode(edges(:,1))) - ...
                        self.world.environment.Nodes.terrain(...
                        self.world.environment.findnode(edges(:,2))));
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
            self.node = target;
        end
    end
end

