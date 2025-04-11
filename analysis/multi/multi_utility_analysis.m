function multi_utility_analysis(missions, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    % ===========================
    
    panel = gui.RightPanel;
    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";
    
    experiments = fields(missions);
    colors = distinguishable_colors(length(experiments), 'white');
    pred_u = {};
    cont_u = {};

    for j = 1:length(experiments)
        M = missions.(experiments{j});
        pred_data = timetable(duration.empty(0,1), [], 'VariableNames', {'data'});
        cont_data = timetable(duration.empty(0,1), [], 'VariableNames', {'data'});
        for k = 1:length(M)
            mission = M(k);
            history = mission.history.pp;
            history.partial_u = nan(height(history), 1);
            for i = 1:height(history)
                robot = mission.robots([mission.robots.id] == history.robot(i));
                pp = robot.pp_outputs(history.Time(i));
                if ~pp.charge_flag
                    idx = find(pp.u == pp.cache.u, 1, 'first');
                    t_mcdm = pp.cache.t_mcdm{idx};
                    u_map = pp.cache.u_map{idx};
                    u_search = pp.cache.u_search{idx};
                    % calculate partial mcdm output
                    u = mcdm(robot.mission.mcdm, t_mcdm, u_map, u_search);
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
            % separate data into groups for boxplot per each charging cycle
            history.groups = nan(height(history), 1);
            for i = 1:length(mission.robots)
                
            end
        end
    end

    % start creating plots
    delete(panel.Children);
    layout = tiledlayout(panel, 2, 1); 

    new_ticks = 0:x_label_interval:mission.time;
    new_ticks.Format = 'hh:mm';

    % plot victim discovery per experiment
    ax = nexttile(layout);
    hold(ax, 'on');

    for i = 1:length(experiments)
        % mean
        plot(ax, count{i}.Time, count{i}.mean, ...
            'Color', colors(i, :));
        % max
        plot(ax, count{i}.Time, count{i}.max, ...
            '-', 'Color', [colors(i, :), 0.3]);
        % min
        plot(ax, count{i}.Time, count{i}.min, ...
            '-', 'Color', [colors(i, :), 0.3]);
        % region
        p = patch(ax, [count{i}.Time' fliplr(count{i}.Time')], ...
            [count{i}.max' fliplr(count{i}.min')], ...
            colors(i, :));
        set(p, 'FaceAlpha', 0.3);
        set(p, 'EdgeColor', 'none');
    end

    legend_entries = {};
    for i = 1:length(experiments)
        legend_entries{end+1} = scatter(ax, nan, nan, ...
            'MarkerEdgeColor', colors(i,:), ...
            'MarkerFaceColor', colors(i,:), ...
            'Marker', 'square');
    end
    legend([legend_entries{:}], cellfun(@(x) replace(string(x), '_', ' '), experiments)', ...
        'Location', 'bestoutside');

    xticks(ax, new_ticks);
    set(ax, 'XTickLabel', {' '});
    title(ax, 'number of unique victim detections');
    grid(ax, 'on');
    hold(ax, 'off');

    % plot victim health distribution per detection
    ax = nexttile(layout);
    hold(ax, 'on');
    
    for i = 1:length(experiments)
        health_data = health{i};
        scatter(ax, health_data.Time, health_data.health, 75, ...
               'MarkerFaceColor', colors(i, :), ...
               'MarkerEdgeColor', colors(i, :), ...
               'Marker', '.');
    end
    legend(ax, cellfun(@(x) replace(string(x), '_', ' '), experiments), "Location", "best");

    xticks(ax, new_ticks);
    xlim(ax, [new_ticks(1) new_ticks(end)]);
    xticklabels(ax, string(new_ticks));
    xtickangle(ax, 90);
    
    xlabel(ax, 'time (hh:mm)');
    ylabel(ax, '%', "Rotation", 0);
    title(ax, "health status of detected victims", 'Interpreter', 'none');

    hold(ax, 'off');

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

