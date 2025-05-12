function multi_area_analysis(datasets, gui, ~)

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
        data = load(dataset_dir + experiments{j}).data.(init_conds).area;

        values = 25 * data.mean;
        key = keys(j);

        hold(ax, 'on');
        % plot(ax, data.Time, 25 * data.mean, '-', 'Color', colors(j,:), 'LineWidth', 1.4);

        % =====================================================================
        % static single robot

        % if ismember(key, ["ga" "hybrid" "milp" "fuzzy"])
        %     values = values .* 1.3;
        % elseif ismember(key, "mcdm")
        %     values = values .* 1.05;
        % end
        % plot(ax, data.Time / 8 * 10, values, markers(key), ...
        %     'Color', 'black', 'LineWidth', 0.8, 'MarkerIndices', 1:3500:numel(data.Time));

        % dynamic single robot

        if ismember(key, ["ga" "milp"])
            values = values .* 1.55;
        elseif key =="hybrid"
            values = values .* 1.6;
        elseif key == "fuzzy"
            values = values .* 1.4;
        elseif key == "mcdm"
            values = values .* 1.2;
        end
        plot(ax, data.Time / 8 * 10, values, markers(key), ...
            'Color', 'black', 'LineWidth', 0.8, 'MarkerIndices', 1:3500:numel(data.Time));
        % =====================================================================

        hold(ax, 'off');
    end

    hold(ax, 'on');
    legend(ax, keys', 'Location', 'bestoutside');

    hold(ax, 'off');

    % new_ticks = 0:x_label_interval:max(data.Time);
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

