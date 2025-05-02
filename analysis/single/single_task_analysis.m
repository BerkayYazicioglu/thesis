function single_task_analysis(mission, gui, robot_id)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    markers = dictionary("map", ".", ...
                         "search", "pentagram");

    % ===========================
    
    panel = gui.RightPanel;

    if nargin == 2
        % need to construct the options
        gui.single_plot_options.Items = ["all" mission.robots.id];
        gui.single_plot_options.Value = "all";
        robot_id = "all";
    else
        robot_id = string(robot_id);
    end

    % filter the history
    if robot_id == "all"
        robot_id = [mission.robots.id]; 
        title_str = 'all robots';
    else
        title_str = robot_id;
    end
    history = mission.history.tasks(ismember(mission.history.tasks.robot, robot_id), :);

    % process task history
    task_history = timetable(duration.empty(0,1), ...
        string.empty, string.empty, string.empty, string.empty, duration.empty, ...
        'VariableNames', {'task_type', 'task_node', 'spawn_robot', 'complete_robot', 't_complete'});

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
                  task_history.complete_robot(idx) = history.robot(i);
            end
        end
        % check spawned tasks
        if ~isempty(history.spawned_nodes{i})
            nodes = history.spawned_nodes{i};
            types = history.spawned_types{i};
            for ii = 1:length(nodes)
                task_history(end+1,:) = {types(ii), ...
                                         nodes(ii), ...
                                         history.robot(i), ...
                                         string(nan), ...
                                         seconds(nan)};
                task_history.Time(end) = history.Time(i);
            end
        end
    end
    
    % start plotting
    delete(panel.Children);
    layout = tiledlayout(panel, 3, 1);
    layout.TileSpacing = 'compact';
    layout.Padding = 'compact';
    new_ticks = 0:x_label_interval:mission.time;
    new_ticks.Format = 'hh:mm';

    % spawns per robot
    ax = nexttile(layout);
    hold(ax, 'on');

    spawn_groups = groupcounts(task_history, ["Time" "task_type"]);
    spawn_groups = rmmissing(spawn_groups);
    task_types = ["map" "search"];
    colors = ["#7E2F8E" "#77AC30"];
    for ii = 1:length(task_types)
        flags = spawn_groups.task_type == task_types(ii);
        scatter(ax, ...
            spawn_groups.Time(flags), spawn_groups.GroupCount(flags), 35, ...
            'MarkerFaceAlpha', 0.9, ...
            'MarkerFaceColor', colors(ii), ...
            'MarkerEdgeColor', colors(ii), ...
            'Marker', markers(task_types(ii)));
    end
    legend(ax, task_types, "Location", "bestoutside");

    xticks(ax, new_ticks);
    xlim(ax, [new_ticks(1) new_ticks(end)]);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    xlabel(ax, 'time (hh:mm)');
    title(ax, 'number of spawned tasks');
    grid(ax, 'on');
    hold(ax, 'off');


    % completed per robot
    ax = nexttile(layout);
    hold(ax, 'on');

    complete_groups = groupcounts(task_history, ["t_complete" "task_type"]);
    complete_groups = rmmissing(complete_groups);
    for ii = 1:length(task_types)
        flags = complete_groups.task_type == task_types(ii);
        scatter(ax, ...
            complete_groups.t_complete(flags), complete_groups.GroupCount(flags), 35, ...
            'MarkerFaceAlpha', 0.9, ...
            'MarkerFaceColor', colors(ii), ...
            'MarkerEdgeColor', colors(ii), ...
            'Marker', markers(task_types(ii)));
    end
    % legend
    legend(ax, task_types, "Location", "bestoutside");

    xticks(ax, new_ticks);
    xlim(ax, [new_ticks(1) new_ticks(end)]);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    xlabel(ax, 'time (hh:mm)');
    title(ax, 'number of completed tasks');
    grid(ax, 'on');
    hold(ax, 'off');

    % histogram of task completion times
    ax = nexttile(layout);
    hold(ax, 'on');

    task_history.t_diff = task_history.t_complete - task_history.Time;
    legend_entries = {};
    for i = 1:length(task_types)
        group = task_history.t_diff(task_history.task_type == task_types(i));
        h = histogram(ax, group, 30, 'FaceAlpha',.5,'EdgeColor','auto');
        legend_entries{end+1} = h;
    end
    legend([legend_entries{:}], task_types);
    xlabel(ax, 'time (hh:mm)');
    title(ax, 'completion times per task type');
    grid(ax, 'on');
    hold(ax, 'off');
    
    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
        for i_ = 1:length(layout.Children)
            if strcmp(get(layout.Children(i_), 'type'), 'axes')
                xlim(layout.Children(i_), [min_lim max_lim]);
            end
        end
    end
end

