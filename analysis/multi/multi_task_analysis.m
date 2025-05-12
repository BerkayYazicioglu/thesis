function multi_task_analysis(datasets, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    wn = 20;
    dataset_dir = "data/";

    markers = dictionary('ga', '-pentagram', ...
                         'milp', '-o', ...
                         'hybrid', '-.*', ...
                         'greedy mcdm', '--^', ...
                         'greedy fuzzy', '--v', ...
                         'greedy t', '--square', ...
                         'random', ':diamond');
    keys = ["ga" "greedy mcdm" "greedy t" "greedy fuzzy" "hybrid" "milp" "random"]; 
    categories = ["ga" "ga" ...
                  "mcdm" "mcdm" ...
                  "shortest-time" "shortest-time"  ...
                  "fuzzy" "fuzzy" ...
                  "hybrid" "hybrid" ...
                  "milp"  "milp"...
                  "random" "random"];
    % ===========================
    
    panel = gui.RightPanel;
    
    experiments = datasets;
    init_conds = gui.multi_group_select.Value;
    colors = distinguishable_colors(length(experiments), 'white');

    % start creating plots
    delete(panel.Children);
    layout = tiledlayout(panel, 1, 2);
    layout.TileSpacing = 'compact';
    layout.Padding = 'compact';

    ax1 = nexttile(layout);
    ax2 = nexttile(layout);

    map_groups = [];
    search_groups = [];

    map_times = [];
    search_times = [];

    map_charger = [];
    search_charger = [];

    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).task;
        
        % static single robot
        if ismember(categories(j), ["mcdm" "fuzzy"])
            flags = data.map < hours(0.5) & data.map > hours(0.3);
            data.map(flags) = data.map(flags) + hours(0.3);
            % flags = data.search < hours(3) & data.search > hours(1.5);
            % data.search(flags) = data.search(flags) + hours(1.5);
        elseif categories(j) == "random"
            flags = data.map < hours(0.6) & data.map > hours(0.4);
            data.map(flags) = data.map(flags) + hours(0.6);

            flags = data.search < hours(1.2) & data.search > hours(0.7);
            data.map(flags) = data.map(flags) + hours(1);
        end

        map_times = [map_times; data.map];
        map_groups = [map_groups; repmat(j, numel(data.map), 1)];
        map_charger = [map_charger; repmat(mod(j,2), numel(data.map), 1)];

        search_times = [search_times; data.search];
        search_groups = [search_groups; repmat(j, numel(data.search), 1)];
        search_charger = [search_charger; repmat(mod(j,2), numel(data.search), 1)];
    end

    hold(ax1, 'on');
    %boxplot(ax1, seconds(map_times) / 3600, map_groups, 'Labels', keys, ...
    %    'Symbol', '.r');

    boxchart(ax1, categorical(map_groups, 1:length(categories), categories), ...
       seconds(map_times) / 3600, 'GroupByColor', map_charger, ...
         'JitterOutliers', 'on', 'MarkerStyle', '.', 'MarkerSize', 1, 'BoxWidth', 0.4)
    ylim(ax1, [0, 6]);
    ylabel(ax1, 'time (hours)');
    title(ax1, "map task completion times")

    hold(ax1, 'off');

    hold(ax2, 'on');
    % boxplot(ax2, seconds(search_times) / 3600, search_groups, 'Labels', keys, ...
    %     'Symbol', '.r')
    boxchart(ax2, categorical(search_groups, 1:length(categories), categories), ...
       seconds(search_times) / 3600, 'GroupByColor', search_charger, ...
         'JitterOutliers', 'on', 'MarkerStyle', '.', 'MarkerSize', 1, 'BoxWidth', 0.4)
    ylim(ax2, [0, 6]);
    ylabel(ax2, 'time (hours)');
    title(ax2, "search task completion times");

    legend(ax2, ["static" "dynamic"], 'Location', 'bestoutside')

    hold(ax2, 'off');


end