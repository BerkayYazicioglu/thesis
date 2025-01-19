function single_mission_analysis(result_path, gui, robot_id)

    % ========= params ==========
    markers = dictionary("victim", "diamond", ...
                         "map", "o", ...
                         "search", "pentagram", ...
                         "robot", "o", ...
                         "charger", "o");
    c_victim = [1 0 0];
    c_victim_found = [0 0.8 0.2];
    c_tasks = ["green" "magenta"];

    robot_size = 50;
    robot_line = 2;
    charger_color = "black";
    charger_line = 2;
    charger_size = 150;

    addpath('src\gui\');
    % ==========================+

    panel = gui.RightPanel;
    mission = load(result_path +  ...
                   gui.dataset_select.Value + "\" + ...
                   gui.mission_select.Value + "\mission.mat").mission;

    global t
    t = mission.time * gui.range_select.Value(2) / 100;
    t_idx = 1;

    if nargin == 2
        % need to construct the options
        gui.single_plot_options.Items = ["all robots" "charger" mission.robots.id];
        gui.single_plot_options.Value = "all robots";
        robot_id = "all robots";
    else
        robot_id = string(robot_id);
    end

    history = mission.history.tasks;
    % process task history
    task_history = timetable(duration.empty(0,1), ...
        string.empty, string.empty, duration.empty, ...
        'VariableNames', {'task_type', 'task_node', 't_complete'});

    for i = 1:height(history)
        % check completed tasks
        if ~isempty(history.completed_nodes{i})
            nodes = history.completed_nodes{i};
            types = history.completed_types{i};
            for ii = 1:length(nodes)
                  idx = task_history.task_node == nodes(ii) & ...
                        task_history.task_type == types(ii);
                  idx = find(idx);
                  task_history.t_complete(idx) = history.Time(i);
            end
        end
        % check spawned tasks
        if ~isempty(history.spawned_nodes{i})
            nodes = history.spawned_nodes{i};
            types = history.spawned_types{i};
            for ii = 1:length(nodes)
                task_history(end+1,:) = {types(ii), ...
                                         nodes(ii), ...
                                         seconds(inf)};
                task_history.Time(end) = history.Time(i);
            end
        end
    end
    % combine robot histories
    robot_history = timetable();
    for i = 1:length(mission.robots)
        r_history = mission.robots(i).history(:, ["node" "action"]);
        r_history.robot_id = repmat(mission.robots(i).id, height(r_history), 1);
        robot_history = [robot_history;
                         r_history];
    end
    robot_history = sortrows(robot_history, "Time");

    % create plots
    delete(panel.Children);
    ax = axes(panel);
    ax.PlotBoxAspectRatio = [1 0.8 1];
    hold(ax, 'on');
    r_colors = distinguishable_colors(length(mission.robots), ...
        {'white', 'black', 'green', 'red'});
    % environment
    contourf(ax, mission.world.X, mission.world.Y, ...
                 reshape(mission.world.environment.Nodes.terrain, mission.world.size()), ...
                 'FaceAlpha', 0, 'EdgeAlpha', 0.25);  

    % victims
    victims = scatter(ax, ...
                      mission.world.X(str2double([mission.world.victims.node])), ...
                      mission.world.Y(str2double([mission.world.victims.node])), ...
                      30, ...
                      'Marker', markers('victim'), ...
                      'MarkerEdgeColor', 'flat', ...
                      'MarkerFaceColor', 'flat'); 
    victim_colors = repmat(c_victim, length(mission.world.victims), 1);
    mask = [mission.world.victims.t_detected] <= t & ...
           [mission.world.victims.t_detected] >= seconds(0);
    if ~isempty(find(mask, 1))
        victim_colors(mask, :) = repmat(c_victim_found, length(find(mask)), 1);
    end
    set(victims, 'CData', victim_colors);

    % tasks
    task_plots = struct();
    actions = ["map" "search"];
    cur_tasks = task_history(task_history.Time <= t & ...
                             task_history.t_complete > t, :);
    for i = 1:length(actions)
        task_nodes = cur_tasks.task_node(cur_tasks.task_type == actions(i));
        task_plots.(actions(i)) = scatter(ax, ...
            mission.world.X(str2double(task_nodes)), ...
            mission.world.Y(str2double(task_nodes)), ...
            25, ...
            'MarkerFaceAlpha', 0.3, ...
            'MarkerEdgeAlpha', 0.3, ...
            'MarkerFaceColor', c_tasks(i), ...
            'MarkerEdgeColor', c_tasks(i), ...
            'Marker', markers(actions(i)));
    end

    % robots
    for i = 1:length(mission.robots)
        robot = mission.robots(i);
        schedule = robot_history(robot_history.robot_id == robot.id, :);
        prev_schedule = schedule(schedule.Time < t, :);
        prev_idx = find(prev_schedule.action ~= "none", 1, 'last');
        if isempty(prev_idx)
            prev_idx = 1;
        end
        prev_path = prev_schedule.node(prev_idx:end);
        schedule = schedule(find(schedule.Time <= t, 1, 'last'):end, :);
        next_idx = find(schedule.action ~= "none", 1, 'first');
        if ~isempty(next_idx)
            schedule = schedule(1:next_idx, :);
        end
        % plot position
        robot_plots.(robot.id).pos = scatter(ax, ...
            mission.world.X(str2double(schedule.node(1))), ...
            mission.world.Y(str2double(schedule.node(1))), ...
            robot_size, ...
            'MarkerFaceColor', r_colors(i, :), ...
            'MarkerEdgeColor', 'black', ...
            'LineWidth', robot_line, ...
            'Marker', markers('robot'));
        % plot path
        robot_plots.(robot.id).prev_path = plot(ax, ...
            mission.world.X(str2double(prev_path)), ...
            mission.world.Y(str2double(prev_path)), ...
            'LineStyle', ':', ...
            'LineWidth', robot_line,  ...
            'Color', r_colors(i, :));
        robot_plots.(robot.id).next_path = plot(ax, ...
            mission.world.X(str2double(schedule.node)), ...
            mission.world.Y(str2double(schedule.node)), ...
            'LineWidth', robot_line,  ...
            'Color', r_colors(i, :));
        % action
        robot_plots.(robot.id).action = scatter(ax, [], [], ...
            75, ...
            'MarkerFaceAlpha', 0.3, ...
            'MarkerEdgeColor', 'none', ...
            'Marker', 'square');
        action = split(schedule.action(1), '_');
        cur_node = robot.node;
        robot.node = schedule.node(1);
        if action(1) == "map"
            robot.mapper.measure(str2double(action(2)));
            robot.mapper.update_gui(robot_plots.(robot.id).action);
            set(robot_plots.(robot.id).action, 'MarkerFaceColor', 'green');
        elseif action(1) == "search"
            robot.detector.measure(str2double(action(2)));
            robot.detector.update_gui(robot_plots.(robot.id).action);
            set(robot_plots.(robot.id).action, 'MarkerFaceColor', 'magenta');
        else
            set(robot_plots.(robot.id).action, 'XData', [], 'YData', []);
        end
        robot.node = cur_node;
    end
    
    % charger
    cur_charger = mission.charger.history(find(mission.charger.history.Time <= t, 1, 'last'), :);
    if mission.charger.policy.optimizer == "static"
        path_charger = cur_charger.node(1);
    else
        charger_keys = mission.charger.pp_outputs.keys;
        charger_key = find(charger_keys <= cur_charger.Time, 1, 'last');
        path_charger = mission.charger.pp_outputs(charger_keys(charger_key)).p;
    end

    charger_plots.pos = scatter(ax, ...
        mission.world.X(str2double(cur_charger.node(1))), ...
        mission.world.Y(str2double(cur_charger.node(1))), ...
        charger_size, ...
        'LineWidth', charger_line, ...
        'MarkerEdgeColor', charger_color, ...
        'MarkerFaceAlpha', 0, ...
        'Marker', markers("charger"));
    charger_plots.path = plot(ax, ...
        mission.world.X(str2double(path_charger)), ...
        mission.world.Y(str2double(path_charger)), ...
        'LineWidth', charger_line,  ...
        'Color', charger_color);

    % map
    map_nodes = mission.map.Nodes(mission.map.Nodes.time <= t, :);
    if mission.charger.policy.optimizer == "static"
        map_charger = {mission.charger.history.node(1)};
    else 
        map_charger = mission.charger.pp_outputs(charger_keys(charger_key)).candidates; 
    end
    map_plot = imagesc(ax, ...
                      [mission.world.X(1,1) mission.world.X(1,end)], ...
                      [mission.world.Y(1,1) mission.world.Y(end,1)], ...
                      zeros([mission.world.size(), 3]));
    map_c = 0.4 * ones(mission.world.size());
    if robot_id == "all robots"
        % global map
        map_c(str2double(map_nodes.Name)) = 0;
        map_c(str2double(map_nodes.Name(~map_nodes.visible))) = 0.7;
    elseif robot_id == "charger"
        % charger candidate map
        map_c(cellfun(@str2double, map_charger)) = 0;
    else
        % robot map, need to calculate traversability
        robot = mission.robots([mission.robots.id] == robot_id); 
        robot_pos = robot_history(robot_history.robot_id == robot.id, 'node');
        robot_pos = robot_pos.node(find(robot_pos.Time <= t, 1, 'last'));
        robot_map = subgraph(mission.map, map_nodes.Name);
        edge_idx = find(~robot.traversability(robot_map.Edges.EndNodes));
        robot_map = robot_map.rmedge(robot_map.Edges.EndNodes(edge_idx,1), ...
                                     robot_map.Edges.EndNodes(edge_idx,2));
        robot_map = robot_map.rmnode(robot_map.Nodes.Name(~robot_map.Nodes.visible));
        map_c(str2double(robot_map.Nodes.Name)) = 0;
        [bin, ~] = conncomp(robot_map);
        accessible = 0 * bin;
        robot_bin = bin(robot_map.findnode(robot_pos));
        accessible = accessible | (bin == robot_bin);
        map_c(cellfun(@str2double, robot_map.Nodes.Name(~accessible))) = 1;
    end
    set(map_plot, 'AlphaData', map_c);

    % legends
    legend_entries = {};
    for i = 1:length(mission.robots)
        legend_entries{end+1} = robot_plots.(mission.robots(i).id).pos;
    end
    legend_entries{end+1} = charger_plots.pos;
    legend_entries{end+1} = task_plots.map;
    legend_entries{end+1} = task_plots.search;
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', markers('victim'), 'MarkerFaceColor', c_victim, 'MarkerEdgeColor', c_victim);
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', markers('victim'), 'MarkerFaceColor', c_victim_found, 'MarkerEdgeColor', c_victim_found);
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', 'square', 'MarkerFaceColor', 'black', 'MarkerEdgeColor', 'black', ...
        'MarkerFaceAlpha', 0.4, 'MarkerEdgeAlpha', 0.4);
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', 'square', 'MarkerFaceColor', 'black', 'MarkerEdgeColor', 'black', ...
        'MarkerFaceAlpha', 0.7, 'MarkerEdgeAlpha', 0.7);
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', 'square', 'MarkerFaceColor', 'black', 'MarkerEdgeColor', 'black', ...
        'MarkerFaceAlpha', 1, 'MarkerEdgeAlpha', 1);
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', 'square', 'MarkerFaceColor', 'green', 'MarkerFaceAlpha', 0.3, 'MarkerEdgeColor', 'none');
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', 'square', 'MarkerFaceColor', 'magenta', 'MarkerFaceAlpha', 0.3, 'MarkerEdgeColor', 'none');
 
    legend([legend_entries{:}], [mission.robots.id ...
                                 "charger" ...
                                 "mapping tasks" ...
                                 "search tasks" ...
                                 "victim (not found)" ...
                                 "victim (found)" ...
                                 "unmapped" ...
                                 "marked not visible" ...
                                 "marked not reachable" ...
                                 "mapping attempt" ...
                                 "search attempt"], ...
        'Location', 'bestoutside');

    xticks(ax, []);
    xticklabels(ax, []);
    yticks(ax, []); 
    yticklabels(ax, []); 
    title(ax, sprintf("mission progression (map: %s)", robot_id));
    ax.XAxis.Visible = 'off';
    ax.YAxis.Visible = 'off';
    xlabel(ax, sprintf('time %s', string(t)));
    ax.XLabel.Visible = 'on';
    hold(ax, 'off');

    % update function
    function update(t)
        % update victims
        victim_colors = repmat(c_victim, length(mission.world.victims), 1);
        mask = [mission.world.victims.t_detected] <= t & ...
               [mission.world.victims.t_detected] >= seconds(0);
        if ~isempty(find(mask, 1))
            victim_colors(mask, :) = repmat(c_victim_found, length(find(mask)), 1);
        end
        set(victims, 'CData', victim_colors);

        % update tasks
        cur_tasks = task_history(task_history.Time <= t & task_history.t_complete > t, :);
        for j = 1:length(actions)
            task_nodes = cur_tasks.task_node(cur_tasks.task_type == actions(j));
            set(task_plots.(actions(j)), ...
                'XData', mission.world.X(str2double(task_nodes)), ...
                'YData', mission.world.Y(str2double(task_nodes)));
        end
        % update charger
        charger_node = mission.charger.history.node(1);
        if mission.charger.policy.optimizer ~= "static"
            cur_charger = mission.charger.history(find(mission.charger.history.Time <= t, 1, 'last'), :);
            charger_key = find(charger_keys <= cur_charger.Time, 1, 'last');
            path_charger = mission.charger.pp_outputs(charger_keys(charger_key)).p;
            path_charger = path_charger(find(path_charger == cur_charger.node(1)):end);
            charger_node = cur_charger.node(1);
            set(charger_plots.pos, ...
                'XData', mission.world.X(str2double(charger_node)), ...
                'YData', mission.world.Y(str2double(charger_node)));
            set(charger_plots.path, ...
                'XData', mission.world.X(str2double(path_charger)), ...
                'YData', mission.world.Y(str2double(path_charger)));
        end      
        % update robots
        for j = 1:length(mission.robots)
            robot = mission.robots(j);
            schedule = robot_history(robot_history.robot_id == robot.id, :);
            prev_schedule = schedule(schedule.Time < t, :);
            prev_idx = find(prev_schedule.action ~= "none", 1, 'last');
            if isempty(prev_idx)
                prev_idx = 1;
            end
            prev_path = prev_schedule.node(prev_idx:end);
            if ismember(t, schedule.Time)
                row_idx = find(schedule.Time == t);
                schedule_rows = schedule(row_idx, :); 
                flags = ~ismember(schedule_rows.action, ["none" "charge" "charge_done"]);
                if any(flags)
                    schedule = schedule(row_idx(find(flags, 1, 'last')):end, :);
                else
                    schedule = schedule(find(schedule.Time <= t, 1, 'last'):end, :);
                end
            else
                schedule = schedule(find(schedule.Time <= t, 1, 'last'):end, :);
            end 
            next_idx = find(schedule.action ~= "none", 1, 'first');
            if ~isempty(next_idx)
                schedule = schedule(1:next_idx, :);
            end
            if ismember(schedule.action(1), ["charge" "charge_done"])
                set(robot_plots.(robot.id).pos, ...
                'XData', mission.world.X(str2double(charger_node)), ...
                'YData', mission.world.Y(str2double(charger_node)));
            else
                set(robot_plots.(robot.id).pos, ...
                'XData', mission.world.X(str2double(schedule.node(1))), ...
                'YData', mission.world.Y(str2double(schedule.node(1))));
            end
            set(robot_plots.(robot.id).prev_path, ...
                'XData', mission.world.X(str2double(prev_path)), ...
                'YData', mission.world.Y(str2double(prev_path)));
            set(robot_plots.(robot.id).next_path, ...
                'XData', mission.world.X(str2double(schedule.node)), ...
                'YData', mission.world.Y(str2double(schedule.node)));
            
            % get action data
            action = split(schedule.action(1), '_');
            cur_node = robot.node;
            robot.node = schedule.node(1);
            if action(1) == "map"
                robot.mapper.measure(str2double(action(2)));
                robot.mapper.update_gui(robot_plots.(robot.id).action);
            set(robot_plots.(robot.id).action, 'MarkerFaceColor', 'green');
            elseif action(1) == "search"
                robot.detector.measure(str2double(action(2)));
                robot.detector.update_gui(robot_plots.(robot.id).action);
                set(robot_plots.(robot.id).action, 'MarkerFaceColor', 'magenta');
            else
                set(robot_plots.(robot.id).action, 'XData', [], 'YData', []);
            end
            robot.node = cur_node;
        end
        % update maps
        map_nodes = mission.map.Nodes(mission.map.Nodes.time <= t, :);
        if mission.charger.policy.optimizer ~= "static"
            map_charger = mission.charger.pp_outputs(charger_keys(charger_key)).candidates; 
        end
        map_c = 0.4 * ones(mission.world.size());
        if robot_id == "all robots"
            % global map
            map_c(str2double(map_nodes.Name)) = 0;
            map_c(str2double(map_nodes.Name(~map_nodes.visible))) = 0.7;
        elseif robot_id == "charger"
            % charger candidate map
            map_c(cellfun(@str2double, map_charger)) = 0;
        else
            % robot map, need to calculate traversability
            robot = mission.robots([mission.robots.id] == robot_id); 
            robot_pos = robot_history(robot_history.robot_id == robot.id, 'node');
            robot_pos = robot_pos.node(find(robot_pos.Time <= t, 1, 'last'));
            robot_map = subgraph(mission.map, map_nodes.Name);
            edge_idx = find(~robot.traversability(robot_map.Edges.EndNodes));
            robot_map = robot_map.rmedge(robot_map.Edges.EndNodes(edge_idx,1), ...
                                         robot_map.Edges.EndNodes(edge_idx,2));
            robot_map = robot_map.rmnode(robot_map.Nodes.Name(~robot_map.Nodes.visible));
            map_c(str2double(robot_map.Nodes.Name)) = 0;
            [bin, ~] = conncomp(robot_map);
            accessible = 0 * bin;
            robot_bin = bin(robot_map.findnode(robot_pos));
            accessible = accessible | (bin == robot_bin);
            map_c(cellfun(@str2double, robot_map.Nodes.Name(~accessible))) = 1;
        end
        set(map_plot, 'AlphaData', map_c);
        xlabel(ax, sprintf('time %s', string(t)));
    end
        
    % bind forward button
    gui.forward.ButtonPushedFcn = @forward_callback;
    function forward_callback(app, event)
        times = robot_history.Time(~ismember(robot_history.action, ["charge_done" "none"]));
        t_new = times(find(times > t, 1, 'first'));
        if isempty(t_new)
            t_new = t;
        end
        % check if the charger movement is next
        t_charger = mission.charger.history.Time(find(mission.charger.history.Time > t, 1, 'first'));
        t = min(t_charger, t_new);
        update(t);
        gui.range_select.Value(2) = 100 * t / mission.time;
    end

    % bind backward button
    gui.backward.ButtonPushedFcn = @backward_callback;
    function backward_callback(app, event)
        times = robot_history.Time(~ismember(robot_history.action, ["charge_done" "none"]));
        t_new = times(find(times < t, 1, 'last'));
        if isempty(t_new)
            t_new = seconds(0);
        end
        % check if the charger movement is next
        t_charger = mission.charger.history.Time(find(mission.charger.history.Time < t, 1, 'last'));
        t = max(t_charger, t_new);
        update(t);
        gui.range_select.Value(2) = 100 * t / mission.time;
    end

    % bind slider
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        max_lim = mission.time * gui.range_select.Value(2) / 100;
        times = robot_history.Time(robot_history.action ~= "none");
        t = times(find(times <= max_lim, 1, 'last'));
        if isempty(t)
            t = seconds(0);
        end
        update(t);
    end
end

   