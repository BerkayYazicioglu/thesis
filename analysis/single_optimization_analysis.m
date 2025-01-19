function single_optimization_analysis(result_path, gui, robot_id)

    % ========= params ==========
    interval = 5 * seconds(60); % minutes
    x_label_interval = 30 * seconds(60); % minutes
    % ===========================
    
    panel = gui.RightPanel;
    mission = load(result_path +  ...
                   gui.dataset_select.Value + "\" + ...
                   gui.mission_select.Value + "\mission.mat").mission;

    if nargin == 2
        % need to construct the options
        gui.single_plot_options.Items = [mission.robots.id];
        gui.single_plot_options.Value = mission.robots(1).id;
        robot_id = mission.robots(1).id;
    end
    
    % start creating plots
    delete(panel.Children);
    layout = tiledlayout(panel, 4, 1);
    robot = mission.robots([mission.robots.id] == robot_id);
    % iterate through robot timesteps
    ts = robot.pp_outputs.keys;
    t_data = [];
    u_map_data = [];
    u_search_data = []; 
    groups = [];
    for i = 1:length(ts)
        pp = robot.pp_outputs(ts(i));
        if ~pp.charge_flag
            n = length([pp.cache.t{:}]);
            rel_t = [pp.cache.t{:}]' - ts(i);
            t_data = [t_data; 1 - min(rel_t, pp.t_max) ./ pp.t_max];
            u_map_data = [u_map_data; [pp.cache.u_map{:}]'];
            u_search_data = [u_search_data; [pp.cache.u_search{:}]'];
            % calculate which interval the data falls under
            t_group = floor(ts(i) / interval);
            groups = [groups; repmat(t_group * seconds(interval), n, 1)];
        end
    end

    new_ticks = 0:x_label_interval:mission.time;
    new_ticks.Format = 'hh:mm';

    % plot the overall utilities 
    ax = nexttile(layout);
    plot(ax, seconds(ts), arrayfun(@(x) robot.pp_outputs(x).u, ts), "Color", 'red');
    xticks(ax, seconds(new_ticks));
    xlim(ax, seconds([new_ticks(1) new_ticks(end)]));
    set(ax, 'XTickLabel', {' '});
    title(ax, 'MCDM utility outputs');

    % plot sub utility groups
    pos = unique(groups);
    ax = nexttile(layout);
    boxplot(ax, t_data, groups, ...
        'PlotStyle', 'compact', ...
        'Positions', pos);
    xticks(ax, []);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'normalized time utility');
    
    ax = nexttile(layout);
    boxplot(ax, u_map_data, groups, ...
        'PlotStyle', 'compact', ...
        'Positions', pos);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'normalized map utility');

    ax = nexttile(layout);
    boxplot(ax, u_search_data, groups, ...
        'PlotStyle', 'compact', ...
        'Positions', pos);
    xticks(ax, seconds(new_ticks));
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    set(ax,'TickLength',[0 0]);
    title(ax, 'normalized search utility');
    xlabel(layout, 'time (hh:mm)');
    title(layout, "optimization analysis | " + robot_id, 'Interpreter', 'none');

    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
        for i_ = 1:length(layout.Children)
            if strcmp(get(layout.Children(i_), 'type'), 'axes')
                xlim(layout.Children(i_), seconds([min_lim max_lim]));
            end
        end
    end
end

