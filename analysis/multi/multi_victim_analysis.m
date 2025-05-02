function multi_victim_analysis(datasets, gui, ~)

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
    layout = tiledlayout(panel, 2, 1);
    layout.TileSpacing = 'compact';
    layout.Padding = 'compact';

    ax1 = nexttile(layout);
    ax2 = nexttile(layout);

    title(ax1, 'victim count');
    title(ax2, 'victim health');

    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).victim;
        
        hold(ax1, 'on');
        %y = conv(data.u_action.mean, ones(wn,1)/wn, 'same');
        plot(ax1, data.count.Time, data.count.mean, '-', 'Color', colors(j,:),'LineWidth', 1.4);
        
        hold(ax1, 'off');

        hold(ax2, 'on');
        %y = conv(data.u_mcdm.mean, ones(wn,1)/wn, 'same');
        plot(ax2, data.health.Time, data.health.health, '-', 'Color', colors(j,:),'LineWidth', 1.4);

        hold(ax2, 'off');
    end

    new_ticks = 0:x_label_interval:data.count.Time(end);
    new_ticks.Format = 'hh:mm';

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

        xticks(Ax(j), new_ticks);
        xticklabels(Ax(j), string(new_ticks));
        xtickangle(Ax(j), 90);
        set(Ax(j),'TickLength',[0 0]);
        hold(Ax(j), 'off');
    end


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

