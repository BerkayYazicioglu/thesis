function single_victim_analysis(result_path, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    markers = dictionary("detected", "pentagram", "revisited", ".");

    addpath('src\gui\');
    % ===========================
    
    panel = gui.RightPanel;
    mission = load(result_path +  ...
                   gui.dataset_select.Value + "\" + ...
                   gui.mission_select.Value + "\mission.mat").mission;

    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";

    % go over victims and construct data
    victim_data = timetable();
    for i = 1:length(mission.world.victims)
        victim = mission.world.victims(i);
        if ~isempty(victim.history)
            history = victim.history;
            history.victim_id = i * ones(height(history), 1);
            victim_data = [victim_data; history];
        end
    end
    victim_data = sortrows(victim_data, 'Time');
    
    % detected unique victims
    detected_data = timetable(seconds(zeros(length(mission.robots), 1)), ...
                              [mission.robots.id]', ...
                              zeros(length(mission.robots), 1), ...
                              zeros(length(mission.robots), 1), ...
                              'VariableNames', {'robot_id', 'count', 'total'});
    for i = 1:height(victim_data)
        row = victim_data(i, :);
        if row.status == "detected"
            detected_data(end+1, :) = {row.robot_id, ...
                detected_data.count(find(detected_data.robot_id == row.robot_id, 1, 'last')) + 1, ...
                detected_data.total(end) + 1};
            detected_data.Time(end) = row.Time;
        end
    end
    % pad the end of data with the most recent values
    for i = 1:length(mission.robots)
        robot = mission.robots(i);
        detected_data(end+1, :) = {robot.id, ...
            detected_data.count(find(detected_data.robot_id == robot.id, 1, 'last')), ...
            detected_data.total(end)};
        detected_data.Time(end) = mission.time;
    end

    % start creating plots
    delete(panel.Children);
    layout = tiledlayout(panel, 2, 1); 

    new_ticks = 0:x_label_interval:mission.time;
    new_ticks.Format = 'hh:mm';

    % plot victim discovery per robot
    ax = nexttile(layout);
    hold(ax, 'on');

    for i = 1:length(mission.robots)
        plot(ax, detected_data.Time(detected_data.robot_id == mission.robots(i).id), ...
                 detected_data.count(detected_data.robot_id == mission.robots(i).id));
    end
    if length(mission.robots) > 1
        plot(ax, detected_data.Time, detected_data.total);
        legend(ax, {mission.robots.id "total"}, "Location", "best");
    else
        legend(ax, {mission.robots.id}, "Location", "best");
    end
    xticks(ax, new_ticks);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'number of unique victim detections');
    grid(ax, 'on');
    hold(ax, 'off');

    % plot victim health distribution per detection
    ax = nexttile(layout);
    hold(ax, 'on');
    
    if ~isempty(victim_data)
        robot_ids = unique(victim_data.robot_id);
        c = distinguishable_colors(length(robot_ids), {'white'});
        status_types = ["detected" "revisited"];
        for i = 1:length(robot_ids)
            for ii = 1:length(status_types)
                sub_data = victim_data(victim_data.robot_id == robot_ids(i) & ...
                                       victim_data.status == status_types(ii), :); 
                if ~isempty(sub_data)
                    scatter(ax, sub_data.Time, sub_data.health, 75, ...
                            'MarkerFaceColor', c(i, :), ...
                            'MarkerEdgeColor', c(i, :), ...
                            'Marker', markers(status_types(ii)));
                end
            end
        end
        % connect dots for each individual victim measurement
        victim_ids = unique(victim_data.victim_id);
        for i = 1:length(victim_ids)
            sub_data = victim_data(victim_data.victim_id == victim_ids(i), :);
            if height(sub_data) > 1
                plot(ax, sub_data.Time, sub_data.health, ...
                    ':', 'LineWidth', 0.2, 'Color', [0 0 0 0.5]);
            end
        end
        % plots for legend
        legend_entries = {};
        for i = 1:length(robot_ids)
            legend_entries{end+1} = scatter(ax, nan, nan, ...
                'MarkerEdgeColor', c(i,:), ...
                'MarkerFaceColor', c(i,:), ...
                'Marker', 'square');
        end
        keys = markers.keys;
        for i = 1:length(keys)
            legend_entries{end+1} = scatter(ax, nan, nan, ...
                'MarkerEdgeColor', 'black', 'Marker', markers(keys(i)));
        end
        legend([legend_entries{:}], [robot_ids; markers.keys]);
    end

    xticks(ax, new_ticks);
    xlim(ax, [new_ticks(1) new_ticks(end)]);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    
    xlabel(ax, 'time (hh:mm)');
    ylabel(ax, '%', "Rotation", 0);
    title(ax, "health status of sensed victims", 'Interpreter', 'none');

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

