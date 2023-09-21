%% General class for robots

classdef Robot < handle
    
    properties  
        id string
        color string

        world World
        camera Sensor
        sensor Sensor

        velocity double {mustBeReal, mustBeNonnegative} % m/s
        max_step double {mustBeReal, mustBeNonnegative} % m
        PI_model mamfis;
        measurements table;
        path (1, :) uint16 {mustBeNonNan};

        time double {mustBeNonnegative} = 0; % s
        height double {mustBeNonnegative, mustBeReal} = 0; % m
        path_plan_steps = 7;
        path_plan_lambda = 0.8;
    end
    
    properties (SetObservable)
        node uint16 {mustBeNonnegative} = 1;
    end

    methods
        %% Constructor
        function self = Robot(params, world)
            % parameters
            % id            : robot id
            % color         : plotting color
            % cam_range     : camera range in meters to detect terrain
            % perc_range    : perception range for remote sensing
            % velocity      : travel velocity m/s
            % max_step      : maximum step increase the robot can climb (m)
            % height        : height of the robot above ground (m)
            % PI_model_file : path of the PI model

            self.id = params.id;
            self.color = params.color;

            self.velocity = params.velocity;
            self.max_step = params.max_step;
            self.height = params.height;
            self.PI_model = readfis(params.PI_model_file);

            self.world = world;

            % camera 
            grid_cell_size = world.world_dim ./ double(world.grid_dim);
            cam_params.range = ceil(params.cam_range ./ grid_cell_size(:));
            cam_params.types = {'terrain', 'destruction'};
            cam_params.remote_sensing = false;
            cam_params.color = 	'green';
            self.camera = Sensor(cam_params);

            % perception
            sens_params.range = ceil(params.perc_range ./ grid_cell_size(:));
            sens_params.types = {'population'};
            sens_params.remote_sensing = true;
            sens_params.color = 'magenta';
            self.sensor = Sensor(sens_params);

            addlistener(self, 'node', 'PostSet', @self.measure);
        end


        %% Get measurements every time a new node is reached
        function measure(self, varargin)
            self.camera.measure(self.height, self.node, self.world);
            self.sensor.measure(self.height, self.node, self.world);
            % merge data
            self.measurements = outerjoin(self.camera.measurements, ...
                                          self.sensor.measurements, ...
                                          "Keys", "nodes", "MergeKeys", true);
            % calculate PI
            self.calculate_PI();
            % update tasks
            sensed_nodes = self.sensor.measurements.nodes;
            self.world.environment.Nodes(sensed_nodes, :).task = repmat(Tasks.none, length(sensed_nodes), 1);
        end

        %% Calulcate PI value from a set of measurements
        function calculate_PI(self)
            if ~isempty(self.measurements)
                PI_input = [self.measurements.destruction ...
                            self.measurements.population];
                % treat NaNs as 0.5
                PI_input = fillmissing(PI_input, 'constant', [0.5 0.5]);
                PI_values = evalfis(self.PI_model, PI_input);
                self.measurements.PI = PI_values;
            end
        end

        %% Place robot on the closest point to the given coordinates
        function place(self, pos)
            arguments
                self Robot
                pos (2,1) double {mustBeReal}
            end
            dif_x = abs(self.world.X(1,:) - pos(1));
            dif_y = abs(self.world.Y(:,1) - pos(2));

            [~, i_x] = min(dif_x);
            [~, i_y] = min(dif_y);

            self.node = sub2ind(self.world.grid_dim, i_y, i_x);
        end

        %% Move the robot towards a target node 
        function dt = move(self, target)
            arguments
                self Robot
                target uint16 {mustBeAValidTarget(target, self, 1)}
            end
            dt = 0;
            if target ~= self.node
                edge = findedge(self.world.environment, self.node, target);
                distance = self.world.environment.Edges(edge,:).Weight;
                dt = distance / self.velocity;
                self.time = self.time + dt;
                self.node = target;
            end
        end

        %% Plan a path
        function path_planner(self)
            % find all possible paths of search depth
            tic;

            paths = find_all_paths(self.world.environment, ...
                                   self.node, ...
                                   self.path_plan_steps, ...
                                   @(g,s) uint16(neighbors(g, s)'));

            fprintf('%s found total %.0f paths in %f seconds\n', self.id, length(paths), toc); 
            % evaluate the paths
            self.evaluate_path([], true);
            tic;
            [best_val, best_idx] = max(cellfun(@(path)self.evaluate_path(path, false), paths));
            fprintf('%s evaluated all paths in %f seconds | e(path) = %.2f\n', self.id, toc, best_val);
            self.path = paths{best_idx};
        end

        %% Evaluate a path
        function value = evaluate_path(self, path, clear_lookups)
            persistent c_lookup u_lookup;
            if clear_lookups
                c_lookup = cell2table({[0 0], 0}, 'VariableNames', {'edge', 'capability'});
                u_lookup = cell2table({[0 0], 0, 0}, 'VariableNames', {'edge', 'dt', 'u_0'});
                value = 0;
            else
                u_tot = 0;
                t_elapsed = self.time;
                cur = path(1);
                for i = 2:length(path)
                    % check if the transition capability already calculated
                    [c_flag, c_loc] = ismember([cur path(i)], c_lookup.edge, 'rows');
                    if c_flag
                        c = c_lookup.capability(c_loc);
                    else
                        c = self.capability(cur, path(i));
                        c_lookup(end+1, :) = table([cur path(i)], c, 'VariableNames', {'edge', 'capability'});
                    end
                    if c == 0
                        value = -1;
                        return
                    end
                    % check if the transition utility already calculated
                    [u_flag, u_loc] = ismember([cur path(i)], u_lookup.edge, 'rows');
                    if u_flag
                        u_0 = u_lookup.u_0(u_loc);
                        dt = u_lookup.dt(u_loc);
                        t_elapsed = t_elapsed + dt;
                        if u_0 == 0
                            u = 0;
                        elseif u_0 == 1
                            u = self.path_plan_lambda^(t_elapsed - self.time);
                        else
                            u = self.path_plan_lambda^(t_elapsed - self.time) + u_0;
                        end
                    else
                        [u, dt, u_0] = self.utility(cur, path(i), t_elapsed);
                        t_elapsed = t_elapsed + dt;
                        u_lookup(end+1, :) = table([cur path(i)], dt, u_0, 'VariableNames', {'edge', 'dt', 'u_0'});
                    end

                    u_tot = u_tot + u;
                    cur = path(i);
                end
                value = c * u_tot;
            end
        end

        %% Capability of transitioning between two given nodes
        function c = capability(self, cur, next)
            step = abs(self.world.environment.Nodes.terrain(cur) - ...
                       self.world.environment.Nodes.terrain(next));
            if step > self.max_step 
                c = 0;
            else
                c = 1;
            end
        end

        %% Utility of reaching a node
        function [u, dt, u_0] = utility(self, cur, next, t_elapsed)
           if ismember(cur, self.camera.measurements.nodes) && ...
              ismember(next, self.camera.measurements.nodes)
               % the actual node distance can be calculated
               edge = findedge(self.world.environment, cur, next);
               distance = self.world.environment.Edges(edge,:).Weight;
           else
               % node distance can be estimated
               cur_coord = [self.world.X(cur) self.world.Y(cur)];
               next_coord = [self.world.X(next) self.world.Y(next)];
               distance = norm(cur_coord - next_coord);
           end
           dt = distance / self.velocity;
           t_elapsed = t_elapsed + dt;
           % task based utilities
           if self.world.environment.Nodes.task(next) == Tasks.none
               u = 0;
               u_0 = 0;
           elseif ismember(next, self.measurements.nodes)
               u_0 = self.measurements.PI(self.measurements.nodes == next);
               u = self.path_plan_lambda^(t_elapsed - self.time) + u_0;
           else
               u_0 = 1;
               u = self.path_plan_lambda^(t_elapsed - self.time);
           end
        end

        %% Plotter
        function plot(self, ax)
            self.camera.plot(ax);
            self.sensor.plot(ax);
            plot(ax, self.world.X(self.node), self.world.Y(self.node), '.', 'Color', self.color, 'MarkerSize', 20);
        end
    end
end


