%% Simulation ground truth  

classdef World < handle
    %% Parameters
    properties(Constant)
        data_fields = {'terrain', 'destruction', 'population', 'task'};
    end

    properties
        grid_dim (1,2) uint16 {mustBePositive} = [1 1]; 
        world_dim (1,2) double {mustBeReal, mustBePositive} = [1 1]; % m

        environment graph = graph; % 8-connected graph where the robots move
        X, Y (:, :) double {mustBeReal, mustBeNonNan}; % meshgrid for x,y coordinates (m)
    end

    %% Methods

    methods
        function self = World(params)
            if nargin < 1
                return
            end
            addpath('utils');
            % load from file
            str = fileread("environment/data/"+params.data_file); 
            data = jsondecode(str); 
            
            % constants
            self.world_dim = data.size;
            self.grid_dim = data.grid;
            [self.X, self.Y] = meshgrid(linspace(0, data.size(1), data.grid(1)), ...
                                        linspace(0, data.size(2), data.grid(2)));

            % graph edge function
            function e = edge_fcn(source, target)
                source_vec = [self.X(source)
                              self.Y(source)
                              data.terrain(source)];
                target_vec = [self.X(target)
                              self.Y(target)
                              data.terrain(target)];
                if params.distance_mode == "2d"
                    e = norm(target_vec(1:2) - source_vec(1:2));
                    return
                end
                e = norm(target_vec - source_vec);
            end

            % graph vertex function
            function v = vertex_fcn(nodes)
                N = numel(nodes);
                table_data = cell(N, length(self.data_fields));
                for i = 1:length(self.data_fields)-1
                    field = num2cell(data.(self.data_fields{i}));
                    [table_data{:,i}] = field{:};
                end
                table_data(:, end) = repmat({Tasks.search}, N, 1);
                v = cell2table(table_data, ...
                               'VariableNames', self.data_fields);
            end

            % get the environment graph
            self.environment = create_map(self.grid_dim, @edge_fcn, @vertex_fcn);
        end
       
        %% plotter
        function plot(self, ax)
            % landscape contour
            contourf(ax, self.X, self.Y, reshape(self.environment.Nodes.terrain, self.grid_dim), ...
                     'FaceAlpha', 0.7); 
            hold on;
            % plot PI over threshold
            % PoI_x = self.X(self.PI_map > self.PI_threshold);
            % PoI_y = self.Y(self.PI_map > self.PI_threshold);
            % PoI = self.PI_map(self.PI_map > self.PI_threshold);
            % 
            % wp = scatter(PoI_x, PoI_y, 10, 'red', 'filled');
            % wp.AlphaData = PoI;
            % wp.MarkerFaceAlpha = 'flat';
        end
    end
end


