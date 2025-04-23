classdef World < handle
    %WORLD Simulation ground truth

    properties(Constant)
        features = {'terrain', 'destruction', 'population', 'victim'};
    end

    properties
        victims (1,:) Victim = Victim.empty;
        environment graph = graph; % 8-connected graph where the robots move
        X, Y (:, :) double {mustBeReal, mustBeNonNan}; % meshgrid for x,y coordinates (m)
    end
    
    methods
        %% Constructor
        function obj = World(params)
            %WORLD Construct an instance of this class
            data = jsondecode(fileread(params.data_file)); 
            
            % constants
            [obj.X, obj.Y] = meshgrid(linspace(0, data.size(1), data.grid(1)), ...
                                      linspace(0, data.size(2), data.grid(2)));

            % graph edge function
            function e = edge_fcn(source, target)
                source_vec = [obj.X(source)
                              obj.Y(source)
                              data.terrain(source)];
                target_vec = [obj.X(target)
                              obj.Y(target)
                              data.terrain(target)];
                e = norm(target_vec - source_vec);
            end

            % graph vertex function
            function v = vertex_fcn(nodes)
                N = numel(nodes);
                table_data = cell(N, 1+length(obj.features));
                for i = 1:length(obj.features)-1
                    field = num2cell(data.(obj.features{i}));
                    [table_data{:,i+1}] = field{:};
                end
                table_data(:, 1) = arrayfun(@string, nodes, 'UniformOutput', 0);
                table_data(:, end) = num2cell(zeros(N, 1));
                v = cell2table(table_data, ...
                               'VariableNames', ['Name', obj.features]);
            end

            % get the environment graph
            obj.environment = create_map(obj.size(), @edge_fcn, @vertex_fcn);

            % place victims
            t_max = duration(params.victims.t_max);
            t_min = duration(params.victims.t_min);
            t_var = duration(params.victims.t_var);

            for n = 1:params.victims.count
                % population density as a pdf to sample a victim location
                [x_v, y_v] = pdf2d(1:size(obj.X,2), ...
                                   1:size(obj.Y,1), ...
                                   data.population);
                node = obj.get_id(x_v, y_v);
                % map the mean wrt the destruction value 
                t_avg = t_min + data.destruction(str2double(node)) * (t_max - t_min);
                t0 = t_avg - t_var;
                t1 = t_avg + t_var;
                t = t0 + rand() * (t1 - t0);
                
                obj.victims = [obj.victims Victim(node, n, t)];
                obj.environment.Nodes.victim(str2double(node)) = ...
                    obj.environment.Nodes.victim(str2double(node)) + 1;
            end

            obj.add_gates();
        end

        %% Get node ID from grid index
        function node_id = get_id(obj, i_x, i_y)
            node_id = string(sub2ind(obj.size(), i_y, i_x));
        end

        %% Get world coordinates from node ID
        function co = get_coordinates(obj, node_id)
            [x, y] = ind2sub(obj.size(), str2double(node_id));
            co = [y(:) x(:)];
        end

        %% Get the environment size
        function s = size(obj)
            s = [size(obj.Y,2) size(obj.X,1)];
        end

        %% Add gates
        function add_gates(obj)
            terrain = obj.environment.Nodes.terrain;
            X = obj.X./500 * 99 + 1;
            Y = obj.Y./500 * 99 + 1;

            % zone 1
            paths = {{[15 23], [15 20], [13 15]}, ...
                     {[15 23], [15 20], [19 19]}, ...
                     {[8 31], [20 31]}, ...
                     {[20 42], [19 47]}, ...
                     {[20 50], [24 48]}, ...
                     {[25 51], [19 48]}, ...
                     {[18 49], [11 49], [18 49]}, ...
                     {[10 69], [17 68]}, ...
                     {[5 64], [13 62]}, ...
                     {[42 24], [35, 18]}, ...
                     {[24 13], [33 12]}, ...
                     {[61 60], [66 75]}, ...
                     {[60 76], [63 90]}, ...
                     {[64 43], [69 38]}, ...
                     {[70 55], [64 51]}, ...
                     {[69 27], [73 21]}, ...
                     {[86 96], [86 86], [80 86]}, ...
                     {[83 79], [94 86], [94 95]}, ...
                     {[94 86], [99 87]}, ...
                     };

            for i = 1:length(paths)
                p = paths{i};
                id = obj.get_id([p{1}(1) p{end}(1)], [p{1}(2) p{end}(2)]);
                idx = obj.environment.findnode(id);
                z0 = terrain(idx);
                [x, y] = bresenham(p{1}(1), p{1}(2), p{2}(1), p{2}(2));
                for j = 3:length(p)
                    [x_n, y_n] = bresenham(p{j-1}(1), p{j-1}(2), p{j}(1), p{j}(2));
                    x = [x; x_n(2:end)];
                    y = [y; y_n(2:end)];
                end
                id = obj.get_id(x, y);
                idx = obj.environment.findnode(id);
                z = linspace(z0(1), z0(2), numel(idx));
                terrain(idx) = z; 
            end
            obj.environment.Nodes.terrain = terrain;
        end

        %% Initialize GUI handles
        function handles = init_gui(obj, parent_handle)
            hold(parent_handle, 'on');

            % terrain
            [~, handles.terrain] = ...
                contourf(parent_handle, obj.X, obj.Y, ...
                         reshape(obj.environment.Nodes.terrain, obj.size()), ...
                         'FaceAlpha', 0.7);  
            % destruction
            [~, handles.destruction] = ...
                contourf(parent_handle, obj.X, obj.Y, ...
                         reshape(obj.environment.Nodes.destruction, obj.size()), ...
                         'FaceAlpha', 0.7);  
            % population
            [~, handles.population] = ...
                contourf(parent_handle, obj.X, obj.Y, ...
                         reshape(obj.environment.Nodes.population, obj.size()), ...
                         'FaceAlpha', 0.7);  
            % victim
            handles.victims = scatter(parent_handle, ...
                obj.X(str2double([obj.victims.node])), ...
                obj.Y(str2double([obj.victims.node])), ...
                10, ...
                'Marker', 'd', ...
                'MarkerEdgeColor', 'red', ...
                'MarkerFaceColor', 'flat'); 

            hold(parent_handle, 'off')
        end
    end
end

