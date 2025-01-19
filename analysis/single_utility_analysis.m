function single_utility_analysis(result_path, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes

    addpath("src\utils\");
    % ===========================

    panel = gui.RightPanel;
    mission = load(result_path +  ...
                   gui.dataset_select.Value + "\" + ...
                   gui.mission_select.Value + "\mission.mat").mission;

    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";
    
    % construct data
    history = mission.history.pp;
    history.partial_u = nan(height(history), 1);
    for i = 1:height(history)
        robot = mission.robots([mission.robots.id] == history.robot(i));
        pp = robot.pp_outputs(history.Time(i));
        if ~pp.charge_flag
            idx = find(pp.u == pp.cache.u, 1, 'first');
            rel_t = pp.cache.t{idx} - history.Time(i);
            rel_t = 1 - min(rel_t, pp.t_max) ./ pp.t_max;
            u_map = pp.cache.u_map{idx};
            u_search = pp.cache.u_search{idx};
            % calculate partial mcdm output
            u = mcdm(robot.mission.mcdm, rel_t, u_map, u_search);
            history.partial_u(i) = sum(u(1: min(length(u), robot.policy.control_horizon)));
        end
    end
    % put charging breaks into the data
    for i = 1:length(mission.robots)
        r_history = mission.robots(i).history;
        r_history = r_history(r_history.action == "charge", "node");
        r_history.robot = repmat(mission.robots(i).id, height(r_history), 1);
        r_history.utility = nan(height(r_history), 1);
        r_history.partial_u = nan(height(r_history), 1);
        history = [history; r_history];
    end
    history = sortrows(history, 'Time');

    % start creating plots
    new_ticks = 0:x_label_interval:mission.time;
    new_ticks.Format = 'hh:mm';

    delete(panel.Children);
    layout = tiledlayout(panel, 2, 1);
    layout.TileSpacing = 'compact';
    layout.Padding = 'compact';
    ax = nexttile(layout);
    hold(ax, 'on');

    % prediction horizon utility
    for i = 1:length(mission.robots)
        idx = history.robot == mission.robots(i).id;
        plot(ax, history.Time(idx), history.utility(idx), '.-', ...
            'MarkerSize', 15, 'LineWidth', 0.1);
    end
    legend(ax, [mission.robots.id]);
    xticks(ax, new_ticks);
    xlim(ax, [new_ticks(1) new_ticks(end)]);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'prediction horizon utility');
    grid(ax, 'on');
    hold(ax, 'off');
    
    % control horizon utility
    ax = nexttile(layout);
    hold(ax, 'on');

    % prediction horizon utility
    for i = 1:length(mission.robots)
        idx = history.robot == mission.robots(i).id;
        plot(ax, history.Time(idx), history.partial_u(idx), '.-', ...
            'MarkerSize', 15, 'LineWidth', 0.1);
    end
    legend(ax, [mission.robots.id]);
    xticks(ax, new_ticks);
    xlim(ax, [new_ticks(1) new_ticks(end)]);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    xlabel(ax, 'time (hh:mm)');
    title(ax, 'control horizon utility');
    grid(ax, 'on');
    hold(ax, 'off');

    % bind silder
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(1));
        max_lim = interp1(linspace(0, 100, numel(new_ticks)), new_ticks, gui.range_select.Value(2));
        xlim(ax, [min_lim max_lim]);
    end
end

