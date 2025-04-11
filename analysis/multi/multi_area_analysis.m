function multi_area_analysis(missions, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    % ===========================

    panel = gui.RightPanel;
    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";
    
    experiments = fields(missions);
    colors = distinguishable_colors(length(experiments), 'white');
    % start creating plots
    delete(panel.Children);
    ax = axes(panel);

    for j = 1:length(experiments)
        M = missions.(experiments{j});
        data = M(1).robots(1).history(:, 'mapped_area');
        robots = M(1).robots;
        for k = 2:length(robots)
            data = synchronize(data, robots(k).history(:,'mapped_area'), 'union');
        end

        for k = 2:length(M)
            robots = M(k).robots;
            for i = 1:length(robots)
                data = synchronize(data, robots(i).history(:, 'mapped_area'), 'union'); 
            end
        end

        hold(ax, 'on')
        data = fillmissing(data, 'previous');

        % mean
        plot(ax, data.Time, mean(data{:, :}, 2), ...
            'Color', colors(j, :));
        % max
        plot(ax, data.Time, max(data{:, :}, [], 2), ...
            '-', 'Color', [colors(j, :), 0.3]);
        % min
        plot(ax, data.Time, min(data{:, :}, [], 2), ...
            '-', 'Color', [colors(j, :), 0.3]);
        % region
        p = patch(ax, [data.Time' fliplr(data.Time')], ...
            [max(data{:, :}, [], 2)' fliplr(min(data{:, :}, [], 2)')], ...
            colors(j, :));
        set(p, 'FaceAlpha', 0.3);
        set(p, 'EdgeColor', 'none');

        hold(ax, 'off');
    end

    hold(ax, 'on');
    legend_entries = {};
    for i = 1:length(experiments)
        legend_entries{end+1} = scatter(ax, nan, nan, ...
            'MarkerEdgeColor', colors(i,:), ...
            'MarkerFaceColor', colors(i,:), ...
            'Marker', 'square');
    end

    legend([legend_entries{:}], cellfun(@(x) replace(string(x), '_', ' '), experiments)', ...
        'Location', 'bestoutside');
    hold(ax, 'off');

    new_ticks = 0:x_label_interval:max(data.Time);
    new_ticks.Format = 'hh:mm';
    xticks(ax, new_ticks);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    
    xlabel(ax, 'time (hh:mm)');
    ylabel(ax, 'm^2', "Rotation", 0);
    title(ax, "mapped area", 'Interpreter', 'none');

    grid(ax, 'on');

    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
        xlim(ax, [min_lim max_lim]);
    end
end

