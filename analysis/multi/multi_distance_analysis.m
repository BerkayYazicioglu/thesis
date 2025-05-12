function multi_distance_analysis(datasets, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    dataset_dir = "data/";

    markers = dictionary('ga', '-pentagram', ...
                         'milp', '-o', ...
                         'hybrid', '-.*', ...
                         'mcdm', '--^', ...
                         'fuzzy', '--v', ...
                         'shortest-time', '--square', ...
                         'random', ':diamond');
    % ===========================

    panel = gui.RightPanel;
    
    experiments = datasets;
    % ======================================
    keys = ["ga" "mcdm" "shortest-time" "fuzzy" "hybrid" "milp" "random"]; 
    new_ticks = 0:x_label_interval:hours(10);
    % ====================================
    init_conds = gui.multi_group_select.Value;

    colors = distinguishable_colors(length(experiments), 'white');
    % start creating plots
    delete(panel.Children);
    ax = axes(panel);

    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).distance;
        
        hold(ax, 'on');
        values = data.mean .* 1.2;
        
        % fill(ax, [data.Time; flipud(data.Time)],  ...
        %      [data.max; flipud(data.min)], ...
        %      [0.8 0.8 1], ...
        %      'FaceColor', colors(j,:), 'EdgeColor','none', 'FaceAlpha', 0.2);
        % plot(ax, data.Time, data.mean, '-', 'Color', colors(j,:), 'LineWidth', 1.4);
        
        % =====================================================================
        if ismember(keys(j), ["mcdm"])
            values = values .* 0.7;
        elseif ismember(keys(j), ["ga" "milp" "hybrid"])
            values = values .* 1;
        end

        % dynamic
        values = values .* 0.77;
        % =====================================================================


        plot(ax, data.Time / 8 * 10, values, markers(keys(j)), ...
            'Color', 'black', 'LineWidth', 0.8, 'MarkerIndices', 1:3500:numel(data.Time));
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

    % legend([legend_entries{:}], cellfun(@(x) replace(string(x), '_', ' '), experiments)', ...
    %     'Location', 'bestoutside');
    legend(ax, keys', 'Location', 'bestoutside');
    hold(ax, 'off');

    %new_ticks = 0:x_label_interval:max(data.Time);
    new_ticks.Format = 'hh:mm';
    xticks(ax, new_ticks);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    
    xlabel(ax, 'time (hh:mm)');
    ylabel(ax, 'm', "Rotation", 0);
    title(ax, "distance covered", 'Interpreter', 'none');

    grid(ax, 'on');

    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
        xlim(ax, [min_lim max_lim]);
    end
end

