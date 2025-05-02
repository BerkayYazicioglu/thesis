function multi_optimization_analysis(datasets, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    wn = 20;
    dataset_dir = "data/";
    % ===========================
    
    panel = gui.RightPanel;
    
    experiments = datasets;
    init_conds = gui.multi_group_select.Value;

    colors = distinguishable_colors(length(experiments), 'white');
    % start creating plots
    delete(panel.Children);
    layout = tiledlayout(panel, 3, 1);
    layout.TileSpacing = 'compact';
    layout.Padding = 'compact';

    ax1 = nexttile(layout);
    ax2 = nexttile(layout);
    ax3 = nexttile(layout);

    title(ax1, 'time utilities');
    title(ax2, 'map utilities');
    title(ax3, 'search utilities');

    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).optimization;

        hold(ax1, 'on');
        y = conv(data.t.mean, ones(wn,1)/wn, 'same');
        plot(ax1, data.t.time, y, '-', 'Color', colors(j,:),'LineWidth', 1.4);
        hold(ax1, 'off');

        hold(ax2, 'on');
        y = conv(data.map.mean, ones(wn,1)/wn, 'same');
        plot(ax2, data.map.time, y, '-', 'Color', colors(j,:),'LineWidth', 1.4);
        hold(ax2, 'off');

        hold(ax3, 'on');
        y = conv(data.search.mean, ones(wn,1)/wn, 'same');
        plot(ax3, data.search.time,y, '-', 'Color', colors(j,:),'LineWidth', 1.4);
        hold(ax3, 'off');
    end

    new_ticks = 0:x_label_interval:seconds(data.t.time(end));
    new_ticks.Format = 'hh:mm';

    legend_entries = {};
    Ax = [ax1, ax2, ax3];
    for j = 1:length(Ax)
        hold(Ax(j), 'on');
        for i = 1:length(experiments)
            legend_entries{end+1} = scatter(Ax(j), nan, nan, ...
                'MarkerEdgeColor', colors(i,:), ...
                'MarkerFaceColor', colors(i,:), ...
                'Marker', 'square');
        end
        legend([legend_entries{:}], cellfun(@(x) replace(string(x), '_', ' '), experiments)', ...
            'Location', 'bestoutside');

        xticks(Ax(j), seconds(new_ticks));
        xticklabels(Ax(j), string(new_ticks));
        xtickangle(Ax(j), 90);
        set(Ax(j),'TickLength',[0 0]);
        hold(Ax(j), 'off');
    end

    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        ticks = seconds(new_ticks);
        min_lim = interp1(linspace(0, 100, numel(ticks)), ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(ticks)), ticks, gui.range_select.Value(2));
        for i_ = 1:length(layout.Children)
            if strcmp(get(layout.Children(i_), 'type'), 'axes')
                xlim(layout.Children(i_), [min_lim max_lim]);
            end
        end
    end
end