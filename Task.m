%% Enumaration class for defined USaR tasks

classdef Task 
    %% Parameters
    properties
        type string;
        pi string;
        node double;
        location (1,2); % (m) [x y]
        robot string = "none";
        % assignable logical;
        t_init duration;

        spawn double {mustBeInRange(spawn, 0, 1)};
        kill double {mustBeInRange(kill, 0, 1)};
        weight; % time-dependent weight function
    end

    %% Methods
    methods
        function self = Task(params)
            % params
            % type       -> (str) type of task (explore, search, patrol, charge)
            % pi         -> (str) related sensor 
            % node       -> (int) node on the graph
            % location   -> [x, y] in coordinates
            % spawn      -> (float) threshold for spawning the task
            % kill       -> (float) threshold for killing the task
            % assignable -> [1,|robots|] logical array of assignable robots
            % t_init     -> (duration) spawn time

            self.type = params.type;
            self.node = params.node;
            self.location = params.location;
            self.pi = params.pi;

            % self.assignable = params.assignable;
            self.kill = params.kill;
            self.spawn = params.spawn;
            self.t_init = params.t_init;
        end

        %% Perform the task if applicable
        function outcome = perform_task(self, robot)
            % check if the robot is asssigned to this task 
            if self.robot == robot.id
                % get the capability
                c = self.capability(robot, robot.node);
                if c >= self.kill
                    % the robot is capable to perform the task
                    outcome = true;
                else
                    % the robot is incapable
                    outcome = false;
                end
            else
                outcome = false;
            end
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
                % using the ground truth here to speed up, since errors are
                % not implemented and the map should already contain most
                % of the nodes along the line
                terrain = robot.world.environment.Nodes.terrain(line);
                % remove NaN, estimate that they can be seen
                % terrain = rmmissing(terrain);
                % find if there is a peak higher than the sensor height
                if all(terrain < robot.height + robot.world.environment.Nodes.terrain(node)) 
                    % all points along the line are below current height
                    c = 1 - robot.visible_sensor.epsilon(distance);
                    return
                elseif max(terrain) <= robot.world.environment.Nodes.terrain(node)
                    % find if the there is an obstruction on the sight
                    c = 1 - robot.visible_sensor.epsilon(distance);
                    return
                else
                    c = 0;
                    return
                end
            end
        end
    end
end