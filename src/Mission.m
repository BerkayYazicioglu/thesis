classdef Mission < handle
    %MISSION Mission class containing all robots and tasks
    
    properties
        world World;
        robots (1,:) Robot {mustBeVector} = Robot.empty;
        tasks (1,:) Task {mustBeVector} = Task.empty;
        charger Charger;
        map graph = graph;
        time duration;
        t_end duration;
        PI_model sugfis;
        prediction_errors logical;
        coordination_radius double;
        q_init cell;
        mcdm;
        
        settings;
        coop_outputs;
        gui_update_flag logical = true; 
        all_idle logical = false;

        history; 
    end
    
    methods
        %% Constructor
        function obj = Mission(params, world)
            %MISSION params (struct) ->
            %   t_end       : (duration) mission end time 
            %   robot       : (struct) robot settings
            %   charger     : (struct) charger settings
            %   coordination: (struct) coordination policy settings
            %   tasks       : (struct) task settings

            obj.settings = params;
            obj.world = world;
            obj.t_end = duration(params.t_end);
            obj.time = seconds(0);
            obj.PI_model = readfis(params.PI_model);
            obj.prediction_errors = params.prediction_errors;
            obj.coordination_radius = params.coordination_radius;
            obj.q_init = params.q_init;
            obj.mcdm = table(cellfun(@(x) string(sort(x)), fields(params.mcdm)), ...
                             cellfun(@(x) params.mcdm.(x), fields(params.mcdm)), ...
                             'VariableNames', {'key', 'weight'});

            % history
            obj.coop_outputs = dictionary();
            obj.history.pp = timetable(duration.empty(0,1), ...
                 string.empty, string.empty, [], ...
                'VariableNames', {'robot', ...
                                  'node', ...
                                  'utility'});
            obj.history.tasks = timetable(duration.empty(0,1), ...
                 string.empty, string.empty, string.empty, {}, {}, {}, {},...
                'VariableNames', {'robot', ...
                                  'node', ...
                                  'action', ...
                                  'spawned_types', ...
                                  'completed_types', ...
                                  'spawned_nodes', ...
                                  'completed_nodes'});

            % charger
            obj.charger = Charger(params.charger, ...
                                  obj.world, ...
                                  obj);
            % populate robots
            robot_ids = fields(params.robots);
            for r = 1:length(robot_ids)
                r_params = params.robots.(robot_ids{r});
                r_params.id = robot_ids{r};
                robot = Robot(r_params, obj.world, obj);
                obj.robots = [obj.robots robot];
            end
            for r = 1:length(obj.robots)
                % take mapping measurements for all around
                robot = obj.robots(r);
                if ~ismember("mapper", robot.capabilities)
                    continue
                end
                for a = 1:length(robot.mapper.angles)
                    robot.mapper.measure(a);
                    robot.update_maps();
                    tt =  timetable(seconds(0), robot.node, "map_"+string(a), ...
                        'VariableNames', {'node', 'action'});
                    results.tasks = obj.manage_tasks(robot, tt);
                    obj.log_history(results);
                end
            end

            % plan individual paths for all robots
            for r = 1:length(obj.robots)
                results.pp = obj.robots(r).path_planner();
                if results.pp.charge_flag
                    % no tasks are viable
                    obj.robots(r).schedule = timetable(obj.charger.time, ...
                        obj.robots(r).node, "none", 100, ...
                        'VariableNames', {'node', 'action', 'energy'});
                    obj.robots(r).state = "idle";
                else
                    obj.robots(r).generate_schedule([results.pp.tasks.node], results.pp.actions, seconds(0));
                end
                results.tt.action = "none"; 
                obj.robots(r).log_history(results);
                obj.log_history(results);
            end
            obj.charger.path_planner();
            % check for conflicts
            [conflicts, conflict_schedules] = detect_conflicts(obj);
            if ~isempty(conflicts)
                results.coop = cooperation(obj, conflict_schedules, conflicts);
                obj.log_history(results);
            end
        end

        %% Run a step of the mission
        function run(obj)
            flags = false;

            % get the next time step from the robots
            [~, idx] = min(arrayfun(@(x) x.schedule.Time(1), obj.robots));
            
            charger_flag = obj.charger.schedule.Time(1) < obj.robots(idx).schedule.Time(1);
            if charger_flag
                obj.charger.run();
                obj.time = obj.charger.time;
                flags = true;
            end
            % charger path planning
            if isempty(obj.charger.schedule)
                obj.charger.path_planner();
            end
            if ~charger_flag 
                results = obj.robots(idx).run();
                obj.time = obj.robots(idx).time;
                if isfield(results, 'tasks')
                    flags = true;
                end
                obj.log_history(results);
                obj.charger.msg = "";
            end
            
            obj.gui_update_flag = any(flags);
            % check if all robots are idle
            end_flags = arrayfun(@(x) x.schedule.action(end) == "none", obj.robots) & [obj.robots.state] == "idle";
            obj.all_idle = all([end_flags obj.charger.idle]);
        end

        %% Log history
        function log_history(obj, outputs)
            % path planners
            if isfield(outputs, 'pp')
                if ~outputs.pp.charge_flag
                    obj.history.pp(end+1, :) = {outputs.pp.robot.id, ...
                                                outputs.pp.robot.node, ...
                                                outputs.pp.u};
                    obj.history.pp.Time(end) = outputs.pp.robot.time;
                end
            end
            % tasks
            if isfield(outputs, 'tasks')
                obj.history.tasks(end+1, :) = {outputs.tasks.robot, ...
                                               outputs.tasks.tt.node, ...
                                               outputs.tasks.tt.action, ...
                                               {outputs.tasks.spawned_types}, ...
                                               {outputs.tasks.completed_types}, ...
                                               {outputs.tasks.spawned_nodes}, ...
                                               {outputs.tasks.completed_nodes}};
                obj.history.tasks.Time(end) = obj.time;
            end
            % cooperation
            if isfield(outputs, 'coop')
                obj.coop_outputs(obj.time) = outputs.coop;
            end
        end

        %% Manage the tasks considering the newly added nodes on the map
        function outputs = manage_tasks(obj, robot, tt)
            spawned = [];
            completed = [];

            % find which mapping tasks should be spawned
            A = adjacency(obj.world.environment);
            known = obj.world.environment.findnode(obj.map.Nodes.Name(obj.map.Nodes.visible));
            unknown = obj.world.environment.findnode(setdiff(obj.world.environment.Nodes.Name, ...
                                         obj.map.Nodes.Name(obj.map.Nodes.visible)));
            A_frontier = A(known, unknown);
            frontier = string(known(any(A_frontier, 2)));

            % spawn mapping tasks
            task_nodes = [[obj.tasks.node] ""];
            for i = 1:length(frontier)
                node = frontier(i);
                if eval(obj.settings.tasks.map.pi_init)
                    obj.tasks(end+1) = Task(obj.settings.tasks.map, ...
                                            obj.time, ...
                                            node, ...
                                            obj.robots);
                    spawned(end+1) = length(obj.tasks);
                end
            end

            % spawn search tasks
            if ismember("mapper", robot.capabilities) && ~isempty(robot.mapper.measurements)
                new_measurements = robot.mapper.measurements(robot.mapper.measurements.is_new, :);
                if ~isempty(new_measurements)
                    % compute PI of the newly mapped nodes
                    PIs = evalfis(obj.PI_model, [new_measurements.destruction new_measurements.population]);
                    for i = 1:height(new_measurements)
                        PI = PIs(i);
                        if ~eval(obj.settings.tasks.search.pi_init)
                            continue
                        end
                        % get neighbors on the current mission map
                        node = new_measurements.nodes(i);
                        nodes = [node; obj.map.neighbors(node)];
                        search_nodes = [obj.tasks([obj.tasks.type] == "search").node];
                        for ii = 1:length(nodes)
                            if ~isempty(search_nodes) && ismember(nodes(ii), search_nodes)
                                % there is already a search task there
                                continue
                            end
                            obj.tasks(end+1) = Task(obj.settings.tasks.search, ...
                                                obj.time, ...
                                                nodes(ii), ...
                                                obj.robots);
                            spawned(end+1) = length(obj.tasks);
                        end
                    end
                end
            end
            
            % detect completed tasks and find the attempted task
            action = tt.action.split('_');

            for i = 1:length(obj.tasks)
                node = obj.tasks(i).node;
                type = obj.tasks(i).type;
                % mapping tasks
                if type == action(1)
                    if type == "map"
                        if eval(obj.settings.tasks.map.pi_complete)
                            completed(end+1) = i;
                        end
                    % search tasks
                    elseif type == "search"
                        if eval(obj.settings.tasks.search.pi_complete)
                            completed(end+1) = i;
                        end
                    end
                    % check if the attempted task is found and if the task is
                    % not completed
                    if node == tt.node && ~ismember(i, completed)
                        % update the task cache for further predictions
                        r_type = robot.id.split('_');
                        obj.tasks(i).cache(end+1,:) = {r_type(1), r_type(2), tt.action};
                    end
                end
            end
            % log spawned and completed tasks
            outputs.robot = robot.id;
            outputs.spawned_types = [obj.tasks(spawned).type];
            outputs.completed_types = [obj.tasks(completed).type];
            outputs.spawned_nodes = [obj.tasks(spawned).node];
            outputs.completed_nodes = [obj.tasks(completed).node];
            outputs.tt = tt;

            obj.tasks(completed) = [];
        end

        %% Initialize GUI handles
        function handles = init_gui(obj, parent_handle)
            hold(parent_handle, 'on');
            % map handles
            handles.map = imagesc(parent_handle, ...
                                 [obj.world.X(1,1) obj.world.X(1,end)], ...
                                 [obj.world.Y(1,1) obj.world.Y(end,1)], ...
                                 zeros([obj.world.size(), 3]));
            % task handles
            handles.tasks.map = scatter(parent_handle, [], [], 20, ...
                                        'Marker', 'd', ...
                                        'MarkerEdgeColor', 'green', ...
                                        'MarkerFaceAlpha', 1);
            handles.tasks.search = scatter(parent_handle, [], [], 20, ...
                                        'Marker', 'd', ...
                                        'MarkerEdgeColor', 'magenta', ...
                                        'MarkerFaceAlpha', 1);
            % charger handle
            handles.charger = obj.charger.init_gui(parent_handle);
            hold(parent_handle, 'off');
        end

        %% Update GUI handles
        function update_gui(obj, handles)
            % update map
            c = 0.4 * ones(obj.world.size());
            c(str2double(obj.map.Nodes.Name)) = 0;
            c(str2double(obj.map.Nodes.Name(~obj.map.Nodes.visible))) = 0.7;
            set(handles.map, 'AlphaData', c);
            % update tasks
            if ~isempty(obj.tasks)
                task_types = fields(handles.tasks);
                for i = 1:length(task_types)
                    t = [obj.tasks([obj.tasks.type] == task_types{i})];
                    co = str2double([t.node]);
                    if isempty(t)
                        set(handles.tasks.(task_types{i}), 'XData', [], 'YData', []);
                    else
                        set(handles.tasks.(task_types{i}), ...
                                           'XData', obj.world.X(co), ...
                                           'YData', obj.world.Y(co));
                    end
                end
            end
            % update charger
            obj.charger.update_gui(handles.charger);
        end
    end
end

