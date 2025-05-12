function multi_utility_analysis(datasets, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    wn = 100;
    dataset_dir = "data/";

    keys = ["ga" "mcdm" "shortest-time" "fuzzy" "hybrid" "milp" "random"]; 
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

    title(ax1, 'action utilities');
    title(ax2, 'MCDM utilities');

    action_groups = [];
    mcdm_groups = [];
    
    action_vals = [];
    mcdm_vals = [];

    action_charger = [];
    mcdm_charger = [];

    for j = 1:length(experiments)
        data = load(dataset_dir + experiments{j}).data.(init_conds).utility;
        action_vals = [action_vals; data.u_action];
        action_groups = [action_groups; repmat(j, numel(data.u_action), 1)];
        action_charger = [action_charger; repmat(mod(j,2), numel(data.u_action), 1)];

        mcdm_vals = [mcdm_vals; data.u_mcdm];
        mcdm_groups = [mcdm_groups; repmat(j, numel(data.u_mcdm), 1)];
        mcdm_charger = [mcdm_charger; repmat(mod(j,2), numel(data.u_mcdm), 1)];
    end

    hold(ax1, 'on');

   boxchart(ax1, categorical(action_groups, 1:length(categories), categories), ...
       action_vals, 'GroupByColor', action_charger, ...
         'JitterOutliers', 'on', 'MarkerStyle', '.', 'MarkerSize', 1, 'BoxWidth', 0.4)
   %legend(ax1, ["static" "dynamic"])
   title(ax1, "fuzzy action evaluation")
   

    % boxplot(ax1, action_vals, action_groups, 'Labels', keys, ...
    %     'Symbol', '.r');
    hold(ax1, 'off');

    hold(ax2, 'on');
    % boxplot(ax2, mcdm_vals, mcdm_groups, 'Labels', keys, ...
    %     'Symbol', '.r');

    boxchart(ax2, categorical(mcdm_groups, 1:length(categories), categories), ...
       mcdm_vals, 'GroupByColor', mcdm_charger, ...
         'JitterOutliers', 'on', 'MarkerStyle', '.', 'MarkerSize', 1, 'BoxWidth', 0.4)
   legend(ax2, ["static" "dynamic"], 'Location', 'bestoutside')
   title(ax2, "MCDM action evaluation")
   

    hold(ax2, 'off');


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

