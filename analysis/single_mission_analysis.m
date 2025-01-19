function single_mission_analysis(result_path, gui, robot_id)

    % ========= params ==========
    markers = dictionary("victim", "diamond", ...
                         "map", "o", ...
                         "search", "pentagram");
    c_victim = [1 0 0];
    c_victim_found = [0 0.8 0.2];
    c_tasks = ["green" "magenta"];

    addpath('src\gui\');
    % ==========================+

    panel = gui.RightPanel;
    mission = load(result_path +  ...
                   gui.dataset_select.Value + "\" + ...
                   gui.mission_select.Value + "\mission.mat").mission;

  
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

    t = mission.time;
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
    mask = [mission.world.victims.t_detected] >= seconds(0);
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
        schedule = schedule(find(schedule.Time <= t, 1, 'last'):end, :);
        next_idx = find(schedule.action ~= "none", 1, 'first');
        if ~isempty(next_idx)
            schedule = schedule(1:next_idx, :);
        end
        % plot position
        robot_plots.(robot.id).pos = scatter(ax, ...
            mission.world.X(str2double(schedule.node(1))), ...
            mission.world.Y(str2double(schedule.node(1))), ...
            50, ...
            'MarkerFaceColor', r_colors(i, :), ...
            'MarkerEdgeColor', 'black', ...
            'LineWidth', 2, ...
            'Marker', 'o');
        % plot path
        robot_plots.(robot.id).path = plot(ax, ...
            mission.world.X(str2double(schedule.node)), ...
            mission.world.Y(str2double(schedule.node)), ...
            'LineWidth', 2,  ...
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
    cur_charger = mission.charger.history(find( mission.charger.history.Time <= t, 1, 'last'), :);
    charger_keys = mission.charger.pp_outputs.keys;
    path_charger = find(charger_keys <= cur_charger.Time, 1, 'last');
    path_charger = mission.charger.pp_outputs(charger_keys(path_charger)).p;

    charger_plots.pos = scatter(ax, ...
        mission.world.X(str2double(cur_charger.node(1))), ...
        mission.world.Y(str2double(cur_charger.node(1))), ...
        150, ...
        'MarkerEdgeColor', 'black', ...
        'MarkerFaceAlpha', 0, ...
        'Marker', 'o');
    charger_plots.path = plot(ax, ...
        mission.world.X(str2double(path_charger)), ...
        mission.world.Y(str2double(path_charger)), ...
        'LineWidth', 1,  ...
        'Color', 'black');

    % map
    map_actions = robot_history(robot_history.Time <= t, :); 
    map_actions = map_actions(~ismember(map_actions.action, ["none" "charge" "charge_done"]), :);
    mapped_nodes = [];
    errors = mission.prediction_errors;
    mission.prediction_errors = false;
    for i = 1:height(map_actions)
        action = split(map_actions.action(i), '_');
        if action(1) == "map"
            robot = mission.robots([mission.robots.id] == map_actions.robot_id(i));
            r_node = robot.node;
            robot.node = map_actions.node(i);
            robot.mapper.measure(str2double(action(2)));
            if ~isempty(robot.mapper.measurements.visible)
                mapped_nodes = [mapped_nodes; 
                    robot.mapper.measurements.nodes(robot.mapper.measurements.visible)];
                mapped_nodes = unique(mapped_nodes);
            end
            robot.node = r_node;
        end
    end
    mission.prediction_errors = errors;

    xticks(ax, []);
    xticklabels(ax, []);
    yticks(ax, []); 
    yticklabels(ax, []); 
    title(ax, sprintf("mission progression (map: %s)", title_str));
    ax.XAxis.Visible = 'off';
    ax.YAxis.Visible = 'off';
    xlabel(ax, sprintf('time (%s - %s)', string(mission.history.pp.Time(1)), string(mission.time)));
    ax.XLabel.Visible = 'on';
    hold(ax, 'off');

    
end

