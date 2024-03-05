%% Sensor class for robots
% Assuming circular sensing around the robots

classdef Sensor < handle
    
    properties
        world World;

        max_range double {mustBeNonnegative} %(m) max range 
        node_range (1,2) double {mustBeNonNan} % nodes
        perimeter double 
        FoV (:,3) double {mustBeNonNan} % circular FoV matrix centered around 0,0 and precalculated uncertainties
        FoV_rays cell % ray casted lines towards the perimeter in FoV 
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
        function self = Sensor(params, world)
            % params:
            %   remote_sensing: bool
            %   features: str (corresponding to a column name in World)
            %   range: (1,2) node range in x and y directions 
            %   max_range: (m) max range in meters
            %   color: color on the plot
            %   t_s: (s) sampling time
            %   d_energy: (%/s) energy percentage depletion per second
            addpath('utils');
            
            self.world = world;
      
            self.remote_sensing = params.remote_sensing;
            self.features = params.features;
            self.max_range = params.max_range;
            self.node_range = ceil(params.max_range ./ world.cell_size(:));
            self.color = params.color;
            self.t_s = duration(params.t_s, 'InputFormat', 'mm:ss');
            self.d_energy = params.d_energy;

            % uncertainty score function
            self.epsilon = uncertainty_score(self.remote_sensing ,...
                                             self.max_range);

            % create neighborhood circular FoV matrix and the perimeter
            perimeter = [];
            self.FoV = [];
            for i = -self.node_range(1):self.node_range(1)
                col = [];
                for j = -self.node_range(2):self.node_range(2) 
                    if (i/self.node_range(1))^2 + (j/self.node_range(2))^2 <= 1
                        % calculate distance in meters to the origin
                        dist = vecnorm([j i] .* world.cell_size);
                        col = [col; j i eval(self.epsilon(dist))];
                    end
                end
                self.FoV = [self.FoV; col];
                % min\max of each column should be a point on the perimeter
                if length(col) > 1
                    perimeter = [perimeter;
                                 col(1, 1:2);
                                 col(end, 1:2)];
                else
                    perimeter = [perimeter; col(1:2)];
                end
            end
            % add missed points by tracing the FoV row by row
            for i = -self.node_range(2):self.node_range(2)
                row = self.FoV(self.FoV(:,1) == i, 1:2);
                perimeter = [perimeter;
                             i min(row(:,2));
                             i max(row(:,2))];
            end
            % simplify repeating entries
            self.perimeter = unique(perimeter, 'rows');

            % ray cast to the perimeter points
            self.FoV_rays = cell(1, length(self.perimeter));
            for k = 1:length(self.perimeter)
                % get the line connecting the node to the perimeter point
                [x, y] = bresenham(0, 0, self.perimeter(k, 1), self.perimeter(k, 2));
                % calculate distance in meters to the origin
                dist = vecnorm([x(:) y(:)] .* world.cell_size, 2, 2);
                self.FoV_rays{k} = [x(:) y(:) eval(self.epsilon(dist))];
            end   
        end

        %% Get all the visible nodes in the FoV from given position
        function [visible, uncertainty] = get_visible(self, height, node)
            [node_row, node_col] = ind2sub(self.world.grid_dim, node);
            visible = [];
            uncertainty = [];
            % find points on the perimeter
            for k = 1:length(self.FoV_rays)
                line = self.FoV_rays{k} + [node_row node_col 0];
                % get the valid coordinates
                line = line(all(line(:, 1:2) >= 1 & ...
                                line(:, 1:2) <= self.world.grid_dim, 2), :);
                % convert to nodes
                line_uncertainty = line(:, 3);
                line = sub2ind(self.world.grid_dim, line(:, 1), line(:, 2));
                % find the points along the line
                terrain = self.world.environment.Nodes.terrain(line);
                % find if there is a peak higher than the sensor height
                if all(terrain < height + self.world.environment.Nodes.terrain(node)) 
                    % all points along the line are below current height
                    visible = [visible; line];
                    uncertainty = [uncertainty; line_uncertainty];
                else
                    if length(terrain) >= 3
                        [pks, locs] = findpeaks([terrain(2:end); terrain(end-1)]);
                    else
                        pks = terrain;
                        locs = 1:length(terrain);
                    end
                    % get the first peak and prune the points after that
                    high_pks_locs = locs(pks >= height + self.world.environment.Nodes.terrain(node));
                    if isempty(high_pks_locs)
                        visible = [visible; line];
                        uncertainty = [uncertainty; line_uncertainty];
                    else
                        idx = min(high_pks_locs);
                        visible = [visible;
                                   line(1:idx)];
                        uncertainty = [uncertainty; 
                                       line_uncertainty(1:idx)];
                    end
                end
               
            end
            [visible, idx, ~] = unique(visible);
            uncertainty = uncertainty(idx);
        end

        %% Get all nodes inside the FoV
        function [field, uncertainty] = get_all(self, node)
            [node_row, node_col] = ind2sub(self.world.grid_dim, node);
            % get all FoV nodes wrt current node
            p_row = node_row + self.FoV(:, 1);
            p_col = node_col + self.FoV(:, 2);
            valid_rows = p_row > 0 & p_row <= self.world.grid_dim(1);
            valid_cols = p_col > 0 & p_col <= self.world.grid_dim(2);
            valid = valid_rows & valid_cols;
            p_row = p_row(valid);
            p_col = p_col(valid);

            field = sub2ind(self.world.grid_dim, p_row, p_col);
            uncertainty = self.FoV(valid, 3);
        end

        %% Update measurements 
        function [energy, dt] = measure(self, height, cur_node)
            if self.remote_sensing 
                [nodes, uncertainty] = self.get_all(cur_node);
            elseif height == Inf
                [nodes, uncertainty] = self.get_all(cur_node);
            else
                [nodes, uncertainty] = self.get_visible(height, cur_node);
            end
              
            coordinates = [self.world.X(nodes) self.world.Y(nodes)];
            self.measurements = table(nodes, coordinates);
            for i = 1:length(self.features)
                % TODO: add measurement errors
                data = self.world.environment.Nodes(nodes, self.features{i});
                self.measurements.(self.features{i}) = data.(self.features{i})(:);
            end
            % add uncertainty (aggregated accross features)
            self.measurements.uncertainty = uncertainty(:);
            dt = self.t_s;
            energy = self.d_energy;
        end

        %% Plot
        function plot(self, gui)
            if nargin > 1
                self.handles.fov_world = scatter(gui.world, ...
                                                [], ...
                                                [], ...
                                                7, 'MarkerFaceAlpha', 0.5, ...
                                                'MarkerFaceColor', self.color, ...
                                                'MarkerEdgeColor', self.color);
            end
            if isempty(self.measurements)
                set(self.handles.fov_world, 'XData', [], 'YData', []);
            else
                set(self.handles.fov_world, ...
                'XData', self.measurements.coordinates(:, 1), ...
                'YData', self.measurements.coordinates(:, 2));
            end
        end
    end
end

