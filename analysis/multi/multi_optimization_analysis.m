function multi_optimization_analysis(datasets, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    wn = 20;
    dataset_dir = "data/";

    keys = ["ga" "greedy mcdm" "greedy t" "greedy fuzzy" "hybrid" "milp" "random"]; 
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

    time_groups = [];
    map_groups = [];
    search_groups = [];
    time_vals = [];
    map_vals = []; 
    search_vals = [];


    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).optimization;

        time_vals = [time_vals; data.t.mean];
        time_groups = [time_groups; repmat(j, numel(data.t.mean), 1)];

        map_vals = [map_vals; data.map.mean];
        map_groups = [map_groups; repmat(j, numel(data.map.mean), 1)];

        search_vals = [search_vals; data.search.mean];
        search_groups = [search_groups; repmat(j, numel(data.search.mean), 1)];
        
    end

   hold(ax1, 'on');
    boxplot(ax1, time_vals, time_groups, 'Labels', keys, ...
        'Symbol', '.r');
    hold(ax1, 'off');

    hold(ax2, 'on');
    boxplot(ax2, map_vals, map_groups, 'Labels', keys, ...
        'Symbol', '.r');
    hold(ax2, 'off');

    hold(ax3, 'on');
    boxplot(ax3, search_vals, search_groups, 'Labels', keys, ...
        'Symbol', '.r');
    hold(ax3, 'off');

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