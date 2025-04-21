function multi_victim_analysis(gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    markers = dictionary("detected", "pentagram", "revisited", ".");
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
    layout = tiledlayout(panel, 2, 1); 

    % plot victim discovery per experiment
    ax = nexttile(layout);
    hold(ax, 'on');

    for i = 1:length(experiments)
        data = load(dataset_dir + experiments{i}).data.(init_conds).victim;

        % mean
        plot(ax, data.count.Time, data.count.mean, ...
            'Color', colors(i, :));
        % max
        plot(ax, data.count.Time, data.count.max, ...
            '-', 'Color', [colors(i, :), alpha]);
        % min
        plot(ax, data.count.Time, data.count.min, ...
            '-', 'Color', [colors(i, :), alpha]);
        % region
        p = patch(ax, [data.count.Time' fliplr(data.count.Time')], ...
            [data.count.max' fliplr(data.count.min')], ...
            colors(i, :));
        set(p, 'FaceAlpha', alpha);
        set(p, 'EdgeColor', 'none');
    end

    legend_entries = {};
    for i = 1:length(experiments)
        legend_entries{end+1} = scatter(ax, nan, nan, ...
            'MarkerEdgeColor', colors(i,:), ...
            'MarkerFaceColor', colors(i,:), ...
            'Marker', 'square');
    end
    legend([legend_entries{:}], cellfun(@(x) replace(string(x), '_', ' '), experiments)', ...
        'Location', 'bestoutside');

    new_ticks = 0:x_label_interval:data.count.Time(end);
    new_ticks.Format = 'hh:mm';
    xticks(ax, new_ticks);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'number of unique victim detections');
    grid(ax, 'on');
    hold(ax, 'off');

    % plot victim health distribution per detection
    ax = nexttile(layout);
    hold(ax, 'on');
    
    for i = 1:length(experiments)
        data = load(dataset_dir + experiments{i}).data.(init_conds).victim;

        scatter(ax, data.health.Time, data.health.health, 75, ...
               'MarkerFaceColor', colors(i, :), ...
               'MarkerEdgeColor', colors(i, :), ...
               'Marker', '.');
    end
    legend(ax, cellfun(@(x) replace(string(x), '_', ' '), experiments), "Location", "best");

    xticks(ax, new_ticks);
    xlim(ax, [new_ticks(1) new_ticks(end)]);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    
    xlabel(ax, 'time (hh:mm)');
    ylabel(ax, '%', "Rotation", 0);
    title(ax, "health status of detected victims", 'Interpreter', 'none');

    hold(ax, 'off');

    % bind silder
    % gui.range_select.ValueChangedFcn = @slider_callback;
    % function slider_callback(app, event)
    %     min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
    %     max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
    %     for i_ = 1:length(layout.Children)
    %         if strcmp(get(layout.Children(i_), 'type'), 'axes')
    %             xlim(layout.Children(i_), [min_lim max_lim]);
    %         end
    %     end
    % end
end

