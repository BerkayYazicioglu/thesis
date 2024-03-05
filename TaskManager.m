%% Global set of tasks

classdef TaskManager < handle
    %% Parameters
    properties
        items dictionary = dictionary();
        world World;

        % explore params
        configs struct; % task-dependent configurations
        explore_weight double = 0.1;
    end

    %% Methods
    methods
        function self = TaskManager(params, world)
            if nargin
                self.world = world;
                self.configs = params;
                % generate explore tasks for all graph nodes
                for i = 1:world.environment.numnodes()
                    self.spawn("explore", 0, ...
                               seconds(0), ...
                               num2str(i), ...
                               self.explore_weight);
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

        %% Plotter
        function plot(self, gui)
            persistent handles
            if nargin > 1
                handles.world = scatter(gui.world, ...
                                        [], ...
                                        [], ...
                                        20, ...
                                        'Marker', 'd', ...
                                        'MarkerEdgeColor', 'magenta', ...
                                        'MarkerFaceAlpha', 0);
            end
            locs = [];
            tasks = self.items.values;
            for i = 1:length(tasks)
                if tasks(i).type == "search"
                    locs = [locs; tasks(i).location];
                end
            end
            if ~isempty(locs)
                set(handles.world, 'XData', locs(:, 1), 'YData', locs(:, 2)); 
            end
        end
    end
end