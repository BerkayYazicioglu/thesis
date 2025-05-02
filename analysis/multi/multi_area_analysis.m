function multi_area_analysis(datasets, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    dataset_dir = "data/";
    % ===========================

    panel = gui.RightPanel;
    
    experiments = datasets;
    init_conds = gui.multi_group_select.Value;

    colors = distinguishable_colors(length(experiments), 'white');
    % start creating plots
    delete(panel.Children);
    ax = axes(panel);

    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).area;

        hold(ax, 'on');

        % fill(ax, [data.Time; flipud(data.Time)],  ...
        %      [data.max; flipud(data.min)], ...
        %      [0.8 0.8 1], ...
        %      'FaceColor', colors(j,:), 'EdgeColor','none', 'FaceAlpha', 0.2);
        plot(ax, data.Time, data.mean, '-', 'Color', colors(j,:), 'LineWidth', 1.4);

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

