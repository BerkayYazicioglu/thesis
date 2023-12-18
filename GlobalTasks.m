%% Global set of tasks

classdef GlobalTasks < handle
    %% Parameters
    properties
        items dictionary = dictionary();
        world World;

        % explore params
        exp_pi string;
        exp_spawn_thresh double {mustBeInRange(exp_spawn_thresh, 0, 1)};
        exp_kill_thresh double {mustBeInRange(exp_kill_thresh, 0, 1)};
    end

    %% Methods
    methods
        function self = GlobalTasks(params, world, map, t)
            if nargin
                self.world = world;
                self.exp_pi = params.explore.pi;
                self.exp_spawn_thresh = params.explore.spawn_thresh;
                self.exp_kill_thresh = params.explore.kill_thresh;

                if ~isempty(map.Nodes)
                    % populate exploration tasks
                    tasks = self.spawn_tasks(map, t);
                    self.items = dictionary(nodes, tasks);
                end
            end
        end

        %% Spawn tasks from a given map
        function tasks = spawn_tasks(self, map, t)
            % get the nodes with below threshold uncertainties
            candidates = map.Nodes.Name(map.Nodes.uncertainty >= self.exp_spawn_thresh);
            candidates = cellfun(@str2num, candidates);
            % get new nodes that are not allocated a task 
            if self.items.isConfigured()
                candidates = setdiff(candidates, ...
                                    self.items.keys);
            end
            % create new exploration tasks
            tasks = cell(1, length(candidates));
            for i = 1:length(candidates)
                t_params.type = "explore";
                t_params.t_init = t;
                t_params.node = candidates(i);
                t_params.location = [self.world.X(t_params.node) self.world.Y(t_params.node)];
                tasks{i} = Task(t_params);
            end
        end

        %% Add a task
        function add_task(self, task)
            self.items(task.node) = task;
        end

        %% Remove a task
        function remove_task(self, node)
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
                                        15, ...
                                        'Marker', 'd', ...
                                        'MarkerEdgeColor', 'black', ...
                                        'MarkerFaceAlpha', 0);
            end
            locs = zeros(self.items.numEntries(), 2);
            tasks = self.items.values;
            for i = 1:length(locs)
                locs(i, :) = tasks(i).location;
            end
            set(handles.world, 'XData', locs(:, 1), 'YData', locs(:, 2)); 
        end
    end
end