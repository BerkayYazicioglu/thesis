function single_energy_analysis(mission, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    % ===========================
    
    panel = gui.RightPanel;

    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";
    
    % start creating plots
    delete(panel.Children);
    ax = axes(panel);
    hold(ax, 'on');

    % individual robots
    for i = 1:length(mission.robots)
        plot(ax, mission.robots(i).history.Time, mission.robots(i).history.energy, ...
            'LineWidth', 0.25);
    end
    legend(ax, {mission.robots.id}, "Location", "best");

    new_ticks = 0:x_label_interval:mission.time;
    new_ticks.Format = 'hh:mm';
    xticks(ax, new_ticks);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    
    xlabel(ax, 'time (hh:mm)');
    ylabel(ax, '%', "Rotation", 0);
    title(ax, "energy percentage", 'Interpreter', 'none');

    hold(ax, 'off');

    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
        xlim(ax, [min_lim max_lim]);
    end
end

