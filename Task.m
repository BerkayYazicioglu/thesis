%% Enumaration class for defined USaR tasks

classdef Task 
    %% Parameters
    properties
        type string;
        pi string;
        node char;
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
            % node       -> (char) node on the graph
            % location   -> [x, y] in coordinates
            % t_init     -> (duration) spawn time
            % weight     -> weight value of the task
            % config._.spawn      -> (float) threshold for spawning the task
            % config._.kill       -> (float) threshold for killing the task
            % config._.pi         -> (str) related sensor 
            % assignable -> [1,|robots|] logical array of assignable robots

            self.type = params.type;
            self.node = params.node;
            self.location = params.location;
            self.t_init = params.t_init;
            self.weight = params.weight;

            % self.assignable = params.assignable;
            self.pi = params.configs.pi;
            self.spawn = params.configs.spawn;
            self.kill = params.configs.kill;
        end

        %% Perform the task if applicable
        % simplify -> False if task allocation should be checked
        %          -> True if performing only depends on the capability
        function outcome = perform_task(self, robot, simplify)
            arguments
                self Task
                robot Robot
                simplify logical = false;
            end
            % check if the robot is asssigned to this task 
            if (self.robot == robot.id) || simplify
                % get the capability
                c = self.capability(robot);
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
            arguments
                self Task
                robot Robot
                node char = '';
            end
            if strcmp(node, '')
                % get the measured capability
                if (self.type == "explore") || (self.type == "search")
                    % is the robot node in the current measurements
                    [~,idx] = ismember(str2double(robot.node), ...
                                       robot.(self.pi).measurements.nodes);
                    if idx == 0
                        % node was not measured
                        c = 0;
                        return;
                    else
                        % get the uncertainty of the measurement
                        epsilon = robot.(self.pi).measurements.uncertainty(idx);
                        c = 1 - epsilon;
                        return;
                    end
                end
            else
                % calculate the capability estimation
                if self.type == "explore"
                    % find the direct line from source to target
                    [s_row, s_col] = ind2sub(robot.world.grid_dim, str2double(node));
                    [t_row, t_col] = ind2sub(robot.world.grid_dim, str2double(self.node));
                    point = [t_row t_col] - [s_row s_col];
                    % find the corresponding perimeter point  
                    [~, idx] = ismember(point, robot.(self.pi).perimeter, 'rows');
                    if idx == 0
                        c = 0;
                        return
                    end
                    line = robot.visible_sensor.FoV_rays{idx};
                    epsilon = line(end, 3);
                    line = arrayfun(@num2str, ...
                                    line(:, 1:2), ...
                                    'UniformOutput', ...
                                    false);
                    idx = robot.map.findnode(line);
                    idx = idx(idx ~= 0);
                    terrain = robot.map.Nodes.terrain(idx);
                    if any(terrain > robot.height + robot.world.environment.Nodes.terrain(str2double(node)))
                        % line of sight is obstructed
                        c = 0;
                        return
                    end
                    % line of sight is not obstructed (by the known values)
                    c = 1 - epsilon;
                elseif self.type == "search"
                    % check if the node is in the measurement range
                    [s_row, s_col] = ind2sub(robot.world.grid_dim, str2double(node));
                    [t_row, t_col] = ind2sub(robot.world.grid_dim, str2double(self.node));
                    point = [t_row t_col] - [s_row s_col];
                    [~, idx] = ismember(point, robot.(self.pi).FoV(:,1:2), 'rows');
                    if idx == 0
                        c = 0;
                        return
                    end
                    epsilon = robot.(self.pi).FoV(idx, 3);
                    c = 1 - epsilon;
                end
            end
        end
    end
end