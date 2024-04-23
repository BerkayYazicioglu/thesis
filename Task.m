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

        %% Utility of the task for a robot if performed
        function u = utility(self, robot)
            u = 0;
            if self.type == "explore"
                u = self.weight;
            elseif self.type == "search"
                u = robot.policy.normalization * ...
                    robot.policy.configs.tradeoff * ...
                    self.weight;
            end
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
                [~, idx] = ismember(str2double(self.node), ...
                                    robot.(self.pi).measurements.nodes);
                c = 1 - robot.(self.pi).measurements.uncertainty(idx);
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
    end
end