%% Sensor class for robots
% Assuming circular sensing around the robots

classdef Sensor < handle
    
    properties
        range (1,2) double {mustBeNonNan} % nodes
        FoV (:,2) int16 {mustBeNonNan} % circular FoV matrix centered around 0,0
        perimeter (:,2) int16 {mustBeNonNan} % perimeter of the FoV

        measurements table = table; % container for the current field of vision
        types cell; % type of measurements
        color string;
        remote_sensing logical; % True -> measures all Fov
                                % False -> only measures visible points
    end
   
    methods
        %% Constructor
        function self = Sensor(params)
            % params:
            %   remote_sensing: bool
            %   type: str (corresponding to a column name in World)
            %   range: (1,2) node range in x and y directions 
            %   color: color on the plot

            addpath('utils');
            
            self.remote_sensing = params.remote_sensing;
            self.types = params.types;
            self.range = params.range;
            self.color = params.color;
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
                if all(terrain < height + world.environment.Nodes.terrain(node)) == true
                    % all points along the line are below current height
                    visible = [visible; line];
                else
                    [pks, locs] = findpeaks([terrain(2:end); terrain(end-1)]);
                    % get the first peak and prune the points after that
                    high_pks_locs = locs(pks >= height + world.environment.Nodes.terrain(node));
                    if isempty(high_pks_locs)
                        % if all peaks are below sensor height, include all
                        % (WIP)
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
        function measure(self, height, cur_node, world)
            if self.remote_sensing
                nodes = self.get_all(cur_node, world);
            else
                nodes = self.get_visible(height, cur_node, world);
            end
            coordinates = [world.X(nodes) world.Y(nodes)];
            self.measurements = table(nodes, coordinates);
            for i = 1:length(self.types)
                data = world.environment.Nodes(nodes, self.types{i});
                self.measurements.(self.types{i}) = data.(self.types{i})(:);
            end
        end

        %% Plot
        function plot(self, ax)
            persistent plot_handle;
            if nargin > 1
                plot_handle = scatter(ax, 0, 0, 1, self.color, 'filled');
            end
            if ~isempty(self.measurements)
                set(plot_handle, 'XData', self.measurements.coordinates(:, 1), ...
                                 'YData', self.measurements.coordinates(:, 2), ...
                                 'SizeData', 10, ...
                                 'MarkerFaceAlpha', 0.7);
     
            end
        end
    end
end

