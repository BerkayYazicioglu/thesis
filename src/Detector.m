classdef Detector < handle
    %DETECTOR Detector class for peneterating sensors
    
    properties
        % params fields
        type string; 
        features string; % type of measurements
        max_range double {mustBeNonnegative} % max range in terms of nodes
        t_s duration; % sampling time
        d_energy double {mustBeNonnegative}; % energy depletion (%/s)
        angles (1,:) double {mustBeVector}; % (degrees)
        arc double {mustBePositive}; % (degrees) 
        capability double {mustBeInRange(capability,0,1)}; 
        model fistree;

        FoV cell % area of effect per angle (x, y, distance)
        measurements table = table; % container for the current field of vision
        robot Robot;
    end
    
    methods
        %% Constructor
        function obj = Detector(params, robot)
            %Mapper params (struct)->
            %   type      : sensor type
            %   features  : features measurable by the mapper
            %   max_range : max range in terms of nodes
            %   angles    : (degrees) allowed values of angles
            %   arc       : (degrees) angular arc of the FoV
            %   t_s       : (s) sampling time
            %   d_energy  : (%/s) energy percentage depletion per second
            %   capability: capability rating of the sensor
            %   model     : (sugfis) prediction fuzzy model
            
            obj.features = [cellfun(@(x) x, params.features)];
            obj.max_range = params.max_range;
            obj.t_s = duration(params.t_s, 'InputFormat', 'mm:ss');
            obj.t_s.Format = 's';
            obj.d_energy = params.d_energy;
            obj.angles = deg2rad(cell2mat(params.angles)); 
            obj.arc = deg2rad(params.arc);
            obj.type = params.type;
            obj.capability = params.capability;
            obj.model = load(params.model).fistreemodel;

            obj.robot = robot;

            % create the maximal FoV area
            max_FoV = [];
            for i = -obj.max_range:obj.max_range
                col = [];
                for j = -obj.max_range:obj.max_range 
                    if (i/obj.max_range)^2 + (j/obj.max_range)^2 <= 1 
                        col = [col; j i vecnorm([j i])];
                    end
                end
                max_FoV = [max_FoV; col];
            end

            % find FoV per angle arc
            obj.FoV = cell(1, length(obj.angles));
            for i = 1:length(obj.angles)
                points = [];
                lb = mod(-obj.angles(i)+pi/2 - obj.arc/2, 2*pi);
                ub = mod(-obj.angles(i)+pi/2 + obj.arc/2, 2*pi);
                for k = 1:length(max_FoV)
                    alpha = mod(atan2(max_FoV(k,2), max_FoV(k,1)), 2*pi);
                    if lb <= ub
                        isBetween = alpha >= lb && alpha <= ub;
                    else
                        isBetween = alpha >= lb || alpha <= ub;
                    end
                    if isBetween || obj.arc == 2*pi || all(max_FoV(k,:)==[0 0 0])
                        points = [points; max_FoV(k,:)];
                    end
                end
                obj.FoV{i} = points;
            end
        end
        
        %% Measurement function
        function measure(obj, angle_idx)
            [field, distance] = obj.get_valid_nodes(angle_idx, obj.robot.node);
            % take maeasurements from the field
            obj.measurements = table(string(field), 'VariableNames', {'nodes'});
            for i = 1:length(obj.features)
                obj.measurements.(obj.features(i)) = obj.robot.world.environment.Nodes.(obj.features(i))(str2double(field));
            end
            % add distances 
            obj.measurements.distance = distance(:);
            obj.measurements.angle = repmat(obj.angles(angle_idx), [numel(field) 1]);
            % calculate the probability of detecting 
            destruction = obj.robot.world.environment.Nodes.destruction(str2double(field));
            p = obj.probability(distance(:) ./ obj.max_range, destruction(:));
            % remove undetected
            obj.measurements(rand(size(p)) > p, :) = [];
        end

        %% Get the maximal field of effect from a given node
        function [field, distance] = get_valid_nodes(obj, angle_idx, node)
            co = obj.robot.world.get_coordinates(node);
            % get all FoV wrt current node and angle
            fov = obj.FoV{angle_idx};
            p_row = co(1) + fov(:, 1);
            p_col = co(2) + fov(:, 2);
            world_size = obj.robot.world.size();
            valid_rows = p_row > 0 & p_row <= world_size(1);
            valid_cols = p_col > 0 & p_col <= world_size(2);
            valid = valid_rows & valid_cols;
            p_row = p_row(valid);
            p_col = p_col(valid);
            field = obj.robot.world.get_id(p_row, p_col);
            distance = fov(valid, 3);
        end

        %% Predict the new measurements from a given task point
        function outcomes = predict(obj, task)
            nodes = [];
            actions = [];
            values = [];
            distances = [];
            fov_areas = [];
            robot_type = obj.robot.id.split('_');
            for i = 1:length(obj.angles)
                action = task.type + "_" + string(i);
                % if the robot type-action is cached, the prediction
                % is updated as not possible
                if any(task.cache.robot_type == robot_type(1) & ...
                       task.cache.id == robot_type(2) & ...
                       task.cache.action == action)
                    continue;
                end
                [field, distance] = obj.get_valid_nodes(i, task.node);
                % filter the field that are only in the robot map
                filtered_field = field;
                filtered_field(obj.robot.map.findnode(field) == 0) = task.node;
                destruction = obj.robot.map.Nodes.destruction(obj.robot.map.findnode(filtered_field));
                distances = [distances; distance];
                fov_areas = [fov_areas; repmat(length(obj.FoV{i}), length(field), 1)];
                nodes = [nodes; field];
                actions = [actions; repmat(action, length(field), 1)];
            end
            if ~isempty(nodes)
                cap = repmat(obj.capability, size(distances));
                values = evalfis(obj.model, [destruction ...
                                             distances/obj.max_range ...
                                             cap]);
                values = values ./ fov_areas;
            end
            outcomes = table(nodes, values, actions);
        end

        %% Measurement probability function
        function P = probability(obj, distance, destruction)
            P = ones(size(distance));
            if obj.robot.mission.prediction_errors
                b = 1-1./(exp(-10*(destruction-0.3))+exp(1*(destruction-0.5)));
                k = 5;
                P = 0.8./exp(k*b.*distance) + 0.2;
            end
        end

        %% Initialize GUI handles
        function handle = init_gui(obj, parent_handle)
            hold(parent_handle, 'on');
            % area
            handle = scatter(parent_handle, [], [], 7, ...
                             'MarkerFaceAlpha', 0.5, ...
                             'MarkerFaceColor', 'magenta', ...
                             'MarkerEdgeColor', 'magenta');
            hold(parent_handle, 'off')
        end

        %% Update GUI handles
        function update_gui(obj, handle)
            % update gui from current measurements
            if ~isempty(obj.measurements)
                k = str2double(obj.measurements.nodes);
                set(handle, 'XData', obj.robot.world.X(k), 'YData', obj.robot.world.Y(k));
            else
                set(handle, 'XData', [], 'YData', []);
            end
        end
    end
end

