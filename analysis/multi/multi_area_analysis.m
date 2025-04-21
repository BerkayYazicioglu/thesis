function multi_area_analysis(gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    dataset_dir = "analysis/postprocessing/datasets/";
    alpha = 0.15;
    % ===========================

    panel = gui.RightPanel;
    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";
    
    experiments = gui.dataset_select.Items;
    init_conds = gui.multi_group_select.Value;

    colors = distinguishable_colors(length(experiments), 'white');
    % start creating plots
    delete(panel.Children);
    ax = axes(panel);

    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).area;

        hold(ax, 'on');

        % mean
        plot(ax, data.Time, data.mean, ...
            'Color', colors(j, :));
        % max
        plot(ax, data.Time, data.max, ...
            '-', 'Color', [colors(j, :), alpha]);
        % min
        plot(ax, data.Time, data.min, ...
            '-', 'Color', [colors(j, :), alpha]);
        % region
        p = patch(ax, [data.Time' fliplr(data.Time')], ...
            [data.max' fliplr(data.min')], ...
            colors(j, :));
        set(p, 'FaceAlpha', alpha);
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

