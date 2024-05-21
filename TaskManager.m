%% Global set of tasks

classdef TaskManager < handle
    %% Parameters
    properties
        items dictionary = dictionary();
        world World;

        configs struct; % task-dependent configurations
    end

    %% Methods
    methods
        function self = TaskManager(params, world)
            if nargin
                self.world = world;
                self.configs = params;
                % generate explore tasks for all graph nodes
                for i = 1:world.environment.numnodes()
                    self.spawn("explore", ...
                               0, ...
                               seconds(0), ...
                               num2str(i), ...
                               self.configs.explore.weight);
                end
            end
        end
        
        %% Run for a given action
        function run(self, robot, action, time, simplify)
            arguments
                self TaskManager
                robot Robot
                action string
                time duration
                simplify logical = false;
            end

            if action == "none"
                return
            end
            sensor = self.configs.(action).pi;
            % perform applicable tasks
            for i = 1:length(robot.(sensor).measurements.nodes)
                task_node = num2str(robot.(sensor).measurements.nodes(i));
                if self.items.isKey(task_node)
                    % there is a task at the node, try to perform
                    if self.items(task_node).type == action
                        outcome = self.items(task_node).perform_task(robot, simplify);
                        if outcome
                            % if the task could be done, remove it
                            self.kill(task_node); 
                            if action == "explore"
                                % exploration -> search depending on the PI
                                PI = evalfis(robot.PI_model, ...
                                            [robot.(sensor).measurements.destruction(i) ...
                                             robot.(sensor).measurements.population(i)]);
                                self.spawn("search", ...
                                           PI, ...
                                           time, ...
                                           task_node, ...
                                           PI);
                            end
                        end
                    end
                end
            end
        end

        %% Spawn a task
        function spawn(self, type, condition, t_init, node, weight)
            arguments
                self TaskManager
                type string
                condition double % value to meet for spawning
                t_init duration
                node char
                weight double
            end
            if condition >= self.configs.(type).spawn
                params.type = type;
                params.configs = self.configs.(type);
                params.t_init = t_init;
                params.node = node;
                params.location = [self.world.X(str2double(node)) ...
                                   self.world.Y(str2double(node))];
                params.weight = weight;
                self.items(node) = Task(params);
            end
        end
 
        %% Kill a task
        function kill(self, node)
            if self.items.isKey(node)
                self.items(node) = [];
            end
        end
    end
end