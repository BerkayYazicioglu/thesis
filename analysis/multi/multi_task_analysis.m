function multi_task_analysis(datasets, gui, ~)

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
    layout = tiledlayout(panel, 2, 1);
    layout.TileSpacing = 'compact';
    layout.Padding = 'compact';

    ax1 = nexttile(layout);
    ax2 = nexttile(layout);

    title(ax1, 'map task completion times');
    title(ax2, 'search task completion times');

    for j = 1:length(experiments)
        hold(ax1, 'on');
        data = load(dataset_dir + experiments{j}).data.(init_conds).task;
        [f,xi] = ksdensity(seconds(data.map), 'NumPoints', 500);
        plot(ax1, xi, f, '-', 'Color', colors(j,:),'LineWidth', 1.4);
        hold(ax1, 'off');

        hold(ax2, 'on');
        data = load(dataset_dir + experiments{j}).data.(init_conds).task;
        [f,xi] = ksdensity(seconds(data.search), 'NumPoints', 500);
        plot(ax2, xi, f, '-', 'Color', colors(j,:),'LineWidth', 1.4);
        hold(ax2, 'off');
    end

    legend_entries = {};
    Ax = [ax1, ax2];
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
        hold(Ax(j), 'off');
    end
end