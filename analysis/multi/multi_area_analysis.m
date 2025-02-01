function multi_area_analysis(result_path, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    % ===========================
    
    panel = gui.RightPanel;
    mission = load(result_path +  ...
                   gui.dataset_select.Value + "\" + ...
                   gui.mission_select.Value + "\mission.mat").mission;

    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";
    
    % start creating plots
    delete(panel.Children);
    ax = axes(panel);
    hold(ax, 'on');

    da = (mission.world.X(1,2) - mission.world.X(1,1)) * ...
         (mission.world.Y(2,1) - mission.world.Y(1,1));
    % individual robots
    for i = 1:length(mission.robots)
        plot(ax, mission.robots(i).history.Time, mission.robots(i).history.mapped_area * da);
    end
    % combined
    combined = mission.robots(1).history(:, "mapped_area");
    for i = 2:length(mission.robots)
        combined = outerjoin(combined, mission.robots(i).history(:, "mapped_area"), ...
                            'Keys', 'Time', ...
                            'MergeKeys', true, ...
                            'Type', 'full');
    end
    combined = sortrows(combined, 'Time');
    if length(mission.robots) > 1
        % plot combined
        combined = fillmissing(combined, 'previous', 'DataVariables', @isnumeric);
        combined.total = sum(table2array(combined), 2);
        plot(ax, combined.Time, combined.total * da);
        legend(ax, {mission.robots.id "total"}, "Location", "best");
    else
        legend(ax, {mission.robots.id}, "Location", "best");
    end

    new_ticks = 0:x_label_interval:mission.time;
    new_ticks.Format = 'hh:mm';
    xticks(ax, new_ticks);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    
    xlabel(ax, 'time (hh:mm)');
    ylabel(ax, 'm^2', "Rotation", 0);
    title(ax, "mapped area", 'Interpreter', 'none');

    grid(ax, 'on');
    hold(ax, 'off');

    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
        xlim(ax, [min_lim max_lim]);
    end
end

