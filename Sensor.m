%% Sensor class for robots
% Assuming circular sensing around the robots

classdef Sensor < handle
    
    properties
        max_range double {mustBeNonnegative} %(m) max range 
        range (1,2) double {mustBeNonNan} % nodes
        FoV (:,2) double {mustBeNonNan} % circular FoV matrix centered around 0,0
        perimeter (:,2) double {mustBeNonNan} % perimeter of the FoV
        measurements table = table; % container for the current field of vision
        epsilon; % uncertainty score function

        features cell; % type of measurements
        t_s duration; % sampling time
        d_energy double {mustBeNonnegative}; % energy depletion (%/s)

        color string;
        remote_sensing logical; % True -> measures all Fov
                                % False -> only measures visible points
        handles;
    end
   
    methods
        %% Constructor
        function self = Sensor(params)
            % params:
            %   remote_sensing: bool
            %   features: str (corresponding to a column name in World)
            %   range: (1,2) node range in x and y directions 
            %   max_range: (m) max range in meters
            %   color: color on the plot
            %   t_s: (s) sampling time
            %   d_energy: (%/s) energy percentage depletion per second
            addpath('utils');
            
            self.remote_sensing = params.remote_sensing;
            self.features = params.features;
            self.max_range = params.max_range;
            self.range = params.range;
            self.color = params.color;
            self.t_s = duration(params.t_s, 'InputFormat', 'mm:ss');
            self.d_energy = params.d_energy;

            % create neighborhood circular FoV matrix and the perimeter
            self.perimeter = [];
            self.FoV = [];
            for i = -self.range(1):self.range(1)
                col = [];
                for j = -self.range(2):self.range(2) 
                    if (i/self.range(1))^2 + (j/self.range(2))^2 <= 1
                        col = [col; j i];
                    end
                end
                self.FoV = [self.FoV; col];
                % min\max of each column should be a point on the perimeter
                if length(col) > 1
                    self.perimeter = [self.perimeter;
                                      col(1, :);
                                      col(end, :)];
                else
                    self.perimeter = [self.perimeter; col];
                end
            end
            % add missed points by tracing the FoV row by row
            for i = -self.range(2):self.range(2)
                row = self.FoV(self.FoV(:,1) == i, :);
                self.perimeter = [self.perimeter;
                                  i min(row(:,2));
                                  i max(row(:,2))];
            end
            % simplify repeating entries
            self.perimeter = unique(self.perimeter, 'rows');
            % uncertainty score function
            self.epsilon = uncertainty_score(self.remote_sensing ,...
                                             self.max_range);
        end

        %% Get all the visible nodes in the FoV from given position
        function visible = get_visible(self, height, node, world)
            [node_row, node_col] = ind2sub(world.grid_dim, node);
            visible = [];
            % find points on the perimeter
            for k = 1:length(self.perimeter)
                p_row = node_row + self.perimeter(k, 1);
                p_col = node_col + self.perimeter(k, 2);

                if p_row <= 0 p_row = 1; 
                elseif p_row > world.grid_dim(1) p_row = world.grid_dim(1); end
                if p_col <= 0 p_col = 1; 
                elseif p_col > world.grid_dim(2) p_col = world.grid_dim(2); end

                % get the line connecting the node to the perimeter point
                [x, y] = bresenham(node_row, node_col, p_row, p_col);
                line = sub2ind(world.grid_dim, x(:), y(:));
                % find the points along the line
                terrain = world.environment.Nodes.terrain(line);
                % find if there is a peak higher than the sensor height
                if all(terrain < height + world.environment.Nodes.terrain(node)) 
                    % all points along the line are below current height
                    visible = [visible; line];
                else
                    [pks, locs] = findpeaks([terrain(2:end); terrain(end-1)]);
                    % get the first peak and prune the points after that
                    high_pks_locs = locs(pks >= height + world.environment.Nodes.terrain(node));
                    if isempty(high_pks_locs)
                        visible = [visible; line];
                    else
                        visible = [visible;
                                   line(1:min(high_pks_locs))];
                    end
                end
               
            end
            visible = unique(visible);
        end

        %% Get all nodes inside the FoV
        function field = get_all(self, node, world)
            [node_row, node_col] = ind2sub(world.grid_dim, node);
            % get all FoV nodes wrt current node
            p_row = node_row + self.FoV(:, 1);
            p_col = node_col + self.FoV(:, 2);
            valid_rows = p_row > 0 & p_row <= world.grid_dim(1);
            valid_cols = p_col > 0 & p_col <= world.grid_dim(2);
            p_row = p_row(valid_rows & valid_cols);
            p_col = p_col(valid_rows & valid_cols);
                    
            field = sub2ind(world.grid_dim, p_row, p_col);
        end

        %% Update measurements 
        function [energy, dt] = measure(self, height, cur_node, world)
            if self.remote_sensing
                nodes = self.get_all(cur_node, world);
            else
                nodes = self.get_visible(height, cur_node, world);
            end
            coordinates = [world.X(nodes) world.Y(nodes)];
            self.measurements = table(nodes, coordinates);
            % get node distances from current node
            x = world.X(nodes);
            y = world.Y(nodes);
            d = vecnorm([x(:) y(:)]' - [world.X(cur_node); world.Y(cur_node)]);
            for i = 1:length(self.features)
                % TODO: add measurement errors
                data = world.environment.Nodes(nodes, self.features{i});
                self.measurements.(self.features{i}) = data.(self.features{i})(:);
            end
            % add uncertainty (aggregated accross features)
            self.measurements.uncertainty = eval(self.epsilon(d))';
            dt = seconds(self.t_s);
            energy = dt * self.d_energy;
        end

        %% Plot
        function plot(self, gui)
            if nargin > 1
                self.handles.fov_world = scatter(gui.world, ...
                                                self.measurements.coordinates(:, 1), ...
                                                self.measurements.coordinates(:, 2), ...
                                                7, 'MarkerFaceAlpha', 0.5, ...
                                                'MarkerFaceColor', self.color, ...
                                                'MarkerEdgeColor', self.color);
            end
            set(self.handles.fov_world, ...
                'XData', self.measurements.coordinates(:, 1), ...
                'YData', self.measurements.coordinates(:, 2));
        end
    end
end

