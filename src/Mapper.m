classdef Mapper < handle
    %MAPPER mapping capabilities of a robot

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
        model sugfis;

        FoV cell % area of effect per angle (x, y, distance)
        measurements table = table; % container for the current field of vision
        robot Robot;
    end
    
    methods
        %% Constructor
        function obj = Mapper(params, robot)
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
            obj.model = readfis(params.model);

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
            data = cell(1, length(obj.features) + 5);
            feature_types = cellfun(@(x) "double", obj.features);
            obj.measurements = table('Size', [0, length(data)], ...
                'VariableTypes', ["string", feature_types{:}, "double", "logical", "cell", "logical"], ...
                'VariableNames', ["nodes", obj.features{:}, "distance", "visible", "angle", "is_new"]);
            % find the unobstructed points one by one
            idx = obj.get_visible_nodes(field, obj.robot.node);
            for i = 1:length(field)
                data{1} = field(i);
                for j = 1:length(obj.features)
                    data{j+1} = obj.robot.world.environment.Nodes.(obj.features(j))(str2double(field(i)));
                end
                data{end-3} = distance(i);
                data{end-2} = ismember(i, idx);
                data{end-1} = {obj.angles(angle_idx)};
                % decide if the measurement is new on the mission map
                if isempty(obj.robot.mission.map.Nodes)
                    is_new = data{end-2};
                elseif ismember(field(i), obj.robot.mission.map.Nodes.Name)
                    new_idx = obj.robot.mission.map.findnode(field(i));
                    is_new = data{end-2} & ~obj.robot.mission.map.Nodes.visible(new_idx);
                else
                    is_new = data{end-2};
                end
                data{end} = is_new;
                obj.measurements(end+1, :) = data;
            end
            % calculate the probability of detecting 
            p = obj.probability(distance(:) ./ obj.max_range);
            % remove undetected
            obj.measurements(rand(size(p)) > p, :) = [];
        end

        %% Get the nonobstructed nodes from a given point
        function idx = get_visible_nodes(obj, field, node)
            z = obj.robot.height + obj.robot.world.environment.Nodes.terrain(str2double(node));
            co = obj.robot.world.get_coordinates(node);
            idx = [];
            for i = 1:length(field)
                co_ = obj.robot.world.get_coordinates(field(i)); 
                % get the line connecting the node to the terrain
                [x, y] = bresenham(co(1), co(2), co_(1), co_(2));
                line = str2double(obj.robot.world.get_id(x, y));
                if numel(x) > 2
                    if all(obj.robot.world.environment.Nodes.terrain(line(1:end-1)) <= z)
                        % no obstruction, add the node
                        idx(end+1) = i;
                    end
                else
                    % no obstruction, add the node
                    idx(end+1) = i;
                end
            end
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
                       task.cache.action == action)
                    continue;
                end
                [field, distance] = obj.get_valid_nodes(i, task.node);
                fov_area = length(obj.FoV{i});
                % get rid of nodes that are already in the robot map
                idx = ~ismember(field, obj.robot.map.Nodes.Name);    
                field = field(idx);
                distance = distance(idx);
                nodes = [nodes; field];
                distances = [distances; distance];
                actions = [actions; repmat(action, length(field), 1)];
                fov_areas = [fov_areas; repmat(fov_area, length(field), 1)];
            end
            if ~isempty(nodes)
                cap = repmat(obj.capability, size(distances));
                values = evalfis(obj.model, [distances/obj.max_range cap]);
                % normalize values
                values = values ./ fov_areas;
            end
            outcomes = table(nodes, values, actions);
        end

        %% Measurement probability function
        function P = probability(obj, distance)
            P = ones(size(distance));
            if obj.robot.mission.prediction_errors
                P = 0.4 + 0.6./(1+exp(10*(distance-0.7)));
            end
        end

        %% Initialize GUI handles
        function handle = init_gui(obj, parent_handle)
            hold(parent_handle, 'on');
            % area
            handle = scatter(parent_handle, [], [], 7, ...
                             'MarkerFaceAlpha', 0.5, ...
                             'MarkerFaceColor', 'green', ...
                             'MarkerEdgeColor', 'green');
            hold(parent_handle, 'off')
        end

        %% Update GUI handles
        function update_gui(obj, handle)
            % update gui from current measurements
            if ~isempty(obj.measurements)
                k = str2double(obj.measurements.nodes(obj.measurements.visible));
                set(handle, 'XData', obj.robot.world.X(k), 'YData', obj.robot.world.Y(k));
            else
                set(handle, 'XData', [], 'YData', []);
            end
        end
    end
end

