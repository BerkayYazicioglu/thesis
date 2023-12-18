%% Simulation ground truth  

classdef World < handle
    %% Parameters
    properties(Constant)
        features = {'terrain', 'destruction', 'population', 'victim'};
    end

    properties
        grid_dim (1,2) double; 
        world_dim (1,2) double; % m
        cell_size (1,2) double; % m

        victims (1,:) cell;
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
            data = jsondecode(fileread(params.data_file)); 
            
            % constants
            self.world_dim = data.size;
            self.grid_dim = data.grid;
            self.cell_size = self.world_dim ./ double(self.grid_dim);
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
                table_data = cell(N, 1+length(self.features));
                for i = 1:length(self.features)-1
                    field = num2cell(data.(self.features{i}));
                    [table_data{:,i+1}] = field{:};
                end
                table_data(:, 1) = arrayfun(@num2str, 1:N, 'UniformOutput', 0);
                table_data(:, end) = num2cell(zeros(N, 1));
                v = cell2table(table_data, ...
                               'VariableNames', ['Name', self.features]);
            end

            % get the environment graph
            self.environment = create_map(self.grid_dim, @edge_fcn, @vertex_fcn);

            % place victims
            self.victims = cell(1, params.victims.count);
            for n = 1:params.victims.count
                % population density as a pdf to sample a victim location
                v_v = 0;
                while ismember(v_v, self.environment.Nodes.victim)
                    [x_v, y_v] = pdf2d(1:self.grid_dim(1), ...
                                       1:self.grid_dim(2), ...
                                       data.population);
                    v_v = sub2ind(self.grid_dim, x_v, y_v);
                end
                self.environment.Nodes.victim(v_v) = n;

                v_params.id = n;
                v_params.node = v_v;
                v_params.location = [self.X(1, x_v), ...
                                     self.Y(y_v, 1)];
                [v_params.sigma, ...
                 v_params.sigma_init, ...
                 v_params.sigma_crit] = victim_health(params.victims, data.destruction(v_v));
                
                self.victims{n} = Victim(v_params);
            end
        end

        %% plotter
        function plot(self, t, gui)
            persistent handles
            if nargin > 2
                handles.terrain = contourf(gui.world, ...
                                           self.X, ...
                                           self.Y, ...
                                           reshape(self.environment.Nodes.terrain, self.grid_dim), ...
                                           'FaceAlpha', 0.7); 
                handles.population = contourf(gui.population, ...
                                              self.X, ...
                                              self.Y, ...
                                              reshape(self.environment.Nodes.population, self.grid_dim), ...
                                              'LineWidth', 0.1, ...
                                              'FaceAlpha', 0.7);
                v_loc = zeros(length(self.victims), 2);
                v_size = zeros(length(self.victims), 1);
                for v = 1:length(self.victims)
                    v_loc(v, :) = self.victims{v}.location;
                    v_size(v) = self.victims{v}.size;
                end
                handles.victims = scatter(gui.population, ...
                                          v_loc(:, 1), ...
                                          v_loc(:, 2), ...
                                          v_size, ...
                                          'Marker', 'd', ...
                                          'MarkerEdgeColor', 'black', ...
                                          'MarkerFaceColor', 'flat'); 
            end
            v_color = zeros(length(self.victims), 3);
            for v = 1:length(self.victims)
                victim = self.victims{v};
                s = victim.get_health(t);
                if victim.phi_det
                    color = victim.color_det;
                elseif s > victim.sigma_crit
                    color = victim.color_init;
                else
                    color = victim.color_crit;
                end
                v_color(v, :) = color;
            end
            set(handles.victims, 'CData', v_color) 
    
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


