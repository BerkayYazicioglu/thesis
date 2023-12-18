%% Enumaration class for defined USaR tasks

classdef Task 
    %% Parameters
    properties
        type string;
        node double;
        location (1,2); % (m) [x y]
        robot string = "none";
        % assignable logical;
        t_init duration;

        weight; % time-dependent weight function
    end

    %% Methods
    methods
        function self = Task(params)
            % params
            % type       -> (str) type of task (explore, search, patrol, charge)
            % node       -> (int) node on the graph
            % location   -> [x, y] in coordinates
            % assignable -> [1,|robots|] logical array of assignable robots
            % t_init     -> (duration) spawn time

            self.type = params.type;
            self.node = params.node;
            self.location = params.location;
            % self.assignable = params.assignable;
            self.t_init = params.t_init;
        end

        %% Capability of a robot handling this task from a given node 
        function c = capability(self, robot, node)
            source_vec = [robot.world.X(node)
                          robot.world.Y(node)];
            target_vec = [robot.world.X(self.node)
                          robot.world.Y(self.node)];
            distance = norm(target_vec - source_vec);
            
            if self.type == "explore"
                if distance > robot.visible_sensor.max_range
                    c = 0;
                    return
                end
                % calculate the direct line from source to target
                [source_row, source_col] = ind2sub(robot.world.grid_dim, node);
                [target_row, target_col] = ind2sub(robot.world.grid_dim, self.node);
                [line_x, line_y] = bresenham(source_row, ...
                                             source_col, ...
                                             target_row, ...
                                             target_col);
                line = sub2ind(robot.world.grid_dim, line_x(:), line_y(:));
                % find the points along the line
                terrain = robot.map.Nodes.terrain(line);
                % remove NaN, estimate that they can be seen
                terrain = rmmissing(terrain);
                % find if there is a peak higher than the sensor height
                if all(terrain < robot.height + robot.map.Nodes.terrain(node)) 
                    % all points along the line are below current height
                    c = robot.visible_sensor.epsilon(distance);
                    return
                elseif max(terrain) <= robot.map.Nodes.terrain(node)
                    % find if the there is an obstruction on the sight
                    c = robot.visible_sensor.epsilon(distance);
                    return
                else
                    c = 0;
                    return
                end
            end
        end
    end
end