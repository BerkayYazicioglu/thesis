function single_optimization_analysis(mission, gui, robot_id)

    % ========= params ==========
    interval = 5 * seconds(60); % minutes
    x_label_interval = 30 * seconds(60); % minutes
    % ===========================
    
    panel = gui.RightPanel;

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
    data_types = ["t" "map" "search"];
    data = struct;
    for i = 1:length(data_types)
        data.(data_types(i)).mean = [];
        data.(data_types(i)).p1 = [];
        data.(data_types(i)).p2 = [];
        data.(data_types(i)).t = [];
    end
    for i = 1:length(ts)
        pp = robot.pp_outputs(ts(i));
        if ~pp.charge_flag
            ts_data = {[pp.cache.t_mcdm{:}]' ,...
                       [pp.cache.u_map{:}]' ,...
                       [pp.cache.u_search{:}]'};
            % calculate which interval the data falls under
            for j = 1:length(data_types)
                data.(data_types(j)).mean = [data.(data_types(j)).mean; mean(ts_data{j})];
                p1 = prctile(ts_data{j}, 5);
                p2 = prctile(ts_data{j}, 95);
                data.(data_types(j)).p1 = [data.(data_types(j)).p1; p1];
                data.(data_types(j)).p2 = [data.(data_types(j)).p2; p2];
                data.(data_types(j)).t = [data.(data_types(j)).t; seconds(ts(i))];
            end
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
    ax = nexttile(layout);
    hold(ax, 'on');
    fill(ax, [data.t.t; flipud(data.t.t)],  ...
             [data.t.p2; flipud(data.t.p1)], ...
             [0.8 0.8 1], ...
             'EdgeColor','none', 'FaceAlpha', 0.6);
    plot(ax, data.t.t, data.t.mean, 'b-', 'LineWidth', 1.4);
    xticks(ax, []);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'normalized time utility');
    hold(ax, 'off');
    
    ax = nexttile(layout);
    hold(ax, 'on');
    fill(ax, [data.map.t; flipud(data.map.t)],  ...
             [data.map.p2; flipud(data.map.p1)], ...
             [0.8 0.8 1], ...
             'EdgeColor','none', 'FaceAlpha', 0.6);
    plot(ax, data.map.t, data.map.mean, 'b-', 'LineWidth', 1.4);
    xticks(ax, []);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'normalized map action utility');
    hold(ax, 'off');

    ax = nexttile(layout);
    hold(ax, 'on');
    fill(ax, [data.search.t; flipud(data.search.t)],  ...
             [data.search.p2; flipud(data.search.p1)], ...
             [0.8 0.8 1], ...
             'EdgeColor','none', 'FaceAlpha', 0.6);
    plot(ax, data.search.t, data.search.mean, 'b-', 'LineWidth', 1.4);
    xticks(ax, seconds(new_ticks));
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    set(ax,'TickLength',[0 0]);
    title(ax, 'normalized search utility');
    xlabel(layout, 'time (hh:mm)');
    title(layout, "optimization analysis | " + robot_id, 'Interpreter', 'none');
    hold(ax, 'off');

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

