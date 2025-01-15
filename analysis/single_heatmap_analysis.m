function single_heatmap_analysis(result_path, gui, robot_id)

    % ========= params ==========
    markers = dictionary("victim", "diamond", ...
                         "map", ".", ...
                         "search", "pentagram");
    c_victim = [1 0 0];
    c_victim_found = [0 0.8 0.2];

    addpath('src\gui\');
    % ===========================
    
    panel = gui.RightPanel;
    mission = load(result_path +  ...
                   gui.dataset_select.Value + "\" + ...
                   gui.mission_select.Value + "\mission.mat").mission;

    if nargin == 2
        % need to construct the options
        gui.single_plot_options.Items = ["all" mission.robots.id];
        gui.single_plot_options.Value = "all";
        robot_id = "all";
    else
        robot_id = string(robot_id);
    end
    
    % combine robot histories
    history = timetable();
    for i = 1:length(mission.robots)
        r_history = mission.robots(i).history(:, ["node" "action"]);
        r_history.robot_id = repmat(mission.robots(i).id, height(r_history), 1);
        history = [history;
                   r_history];
    end
    history = sortrows(history, "Time");

    % filter the history
    if robot_id == "all"
        robot_id = [mission.robots.id]; 
        title_str = 'all robots';
    else
        title_str = robot_id;
    end
    history = history(ismember(history.robot_id, robot_id), :);
    
    % start plotting
    delete(panel.Children);
    layout = tiledlayout(panel, 2, 1);
    layout.TileSpacing = 'compact';
    layout.Padding = 'compact';
    
    % traversal heatmap
    ax = nexttile(layout);
    ax.PlotBoxAspectRatio = [1 0.8 1];
    hold(ax, 'on');
    % environment
    contourf(ax, mission.world.X, mission.world.Y, ...
                 reshape(mission.world.environment.Nodes.terrain, mission.world.size()), ...
                 'FaceAlpha', 0);  
    % heatmap
    c = zeros(size(mission.world.X, 1), ...
              size(mission.world.Y, 2));
    [GC,GR] = groupcounts(history.node);
    GR = str2double(GR);
    c(GR) = c(GR) + GC;
    traversal = imagesc(ax, ...
                        [mission.world.X(1,1) mission.world.X(1,end)], ...
                        [mission.world.Y(end,1) mission.world.Y(1,1)], ...
                        flipud(c));
    traversal.AlphaData = 0.75;
    cmap = hsv;
    cmap = cmap(120:256, :);
    cmap(1, :) = [1 1 1];
    clim(ax, [0 max(c, [], 'all')]);
    colormap(ax, cmap);
    cbar = colorbar(ax);
    cbar.Label.String = 'number of visits';
    cbar.TickDirection = 'none';
    xticks(ax, []);
    xticklabels(ax, []);
    yticks(ax, []); 
    yticklabels(ax, []); 
    title(ax, sprintf("traversal heatmap of %s", title_str));
    ax.XAxis.Visible = 'off';
    ax.YAxis.Visible = 'off';
    xlabel(ax, sprintf('time (%s - %s)', string(mission.history.pp.Time(1)), string(mission.time)));
    ax.XLabel.Visible = 'on';
    hold(ax, 'off');

    % task execution map 
    ax = nexttile(layout);
    ax.PlotBoxAspectRatio = [1 0.8 1];
    hold(ax, 'on');
    % environment
    contourf(ax, mission.world.X, mission.world.Y, ...
                 reshape(mission.world.environment.Nodes.terrain, mission.world.size()), ...
                 'FaceAlpha', 0, 'EdgeAlpha', 0.25);  
    % victims
    victims = scatter(ax, ...
                      mission.world.X(str2double([mission.world.victims.node])), ...
                      mission.world.Y(str2double([mission.world.victims.node])), ...
                      30, ...
                      'Marker', markers('victim'), ...
                      'MarkerEdgeColor', 'black', ...
                      'MarkerFaceColor', 'flat'); 
    victim_colors = repmat(c_victim, length(mission.world.victims), 1);
    mask = [mission.world.victims.t_detected] >= seconds(0);
    if ~isempty(find(mask, 1))
        victim_colors(mask, :) = repmat(c_victim_found, length(find(mask)), 1);
    end
    set(victims, 'CData', victim_colors);
    % tasks
    task_history = history(history.action ~= "none", :);
    r_colors = distinguishable_colors(length(robot_id), ...
        {'white', 'black', 'green', 'red'});
    task_plots = struct();
    actions = ["map" "search"];
    for i = 1:length(robot_id)
        robot = mission.robots([mission.robots.id] == robot_id(i));
        for ii = 1:length(actions)
            r_history = task_history(task_history.robot_id == robot.id, :);
            entries = arrayfun(@(x) split(x, "_"), r_history.action, 'UniformOutput', false); 
            entries = cellfun(@(x) x(1), entries); 
            if ~isempty(entries)
                r_history = r_history(entries == actions(ii), :);
                xval = mission.world.X(str2double([r_history.node]));
                yval = mission.world.Y(str2double([r_history.node]));
            else
                xval = nan;
                yval = nan;
            end
            task_plots.(robot.id).(actions(ii)) = scatter(ax, ...
                xval, yval, 35, ...
                'MarkerFaceAlpha', 0.5, ...
                'MarkerFaceColor', r_colors(i, :), ...
                'MarkerEdgeColor', r_colors(i, :), ...
                'Marker', markers(actions(ii)));
        end
    end
    xticks(ax, []);
    xticklabels(ax, []);
    yticks(ax, []); 
    yticklabels(ax, []);
    ax.XAxis.Visible = 'off';
    ax.YAxis.Visible = 'off';
    xlabel(ax, sprintf('time (%s - %s)', string(mission.history.pp.Time(1)), string(mission.time)));
    ax.XLabel.Visible = 'on';
    title(ax, sprintf("task execution heatmap of %s", title_str));
    % construct legend
    legend_entries = {};
    for i = 1:length(robot_id)
        legend_entries{end+1} = scatter(ax, nan, nan, ...
            'MarkerEdgeColor', r_colors(i,:), ...
            'MarkerFaceColor', r_colors(i,:), ...
            'Marker', 'square');
    end
    for i = 1:length(actions)
        legend_entries{end+1} = scatter(ax, nan, nan, ...
            'MarkerEdgeColor', 'black', 'Marker', markers(actions(i)));
    end
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', markers('victim'), 'MarkerFaceColor', c_victim, 'MarkerEdgeColor', 'black');
    legend_entries{end+1} = scatter(ax, nan, nan, ...
        'Marker', markers('victim'), 'MarkerFaceColor', c_victim_found, 'MarkerEdgeColor', 'black');
    legend([legend_entries{:}], [robot_id actions "victim (not found)" "victim (found)"], ...
        'Location', 'bestoutside');
    hold(ax, 'off');


    % bind slider
    gui.range_select.ValueChangedFcn = @slider_callback;
    function slider_callback(app, event)
        min_lim = mission.time * gui.range_select.Value(1) / 100;
        max_lim = mission.time * gui.range_select.Value(2) / 100;
        history_ = history((history.Time >= min_lim) & (history.Time <= max_lim), :);

        % traversal
        c_ = zeros(size(mission.world.X, 1), ...
                   size(mission.world.Y, 2));
        [GC_,GR_] = groupcounts(history_.node);
        GR_ = str2double(GR_);
        c_(GR_) = c_(GR_) + GC_;
        set(traversal, "CData", flipud(c_));
        xlabel(traversal.Parent, sprintf('time (%s - %s)', string(min_lim), string(max_lim)));

        % victims
        victim_colors_ = repmat(c_victim, length(mission.world.victims), 1);
        mask_ = [mission.world.victims.t_detected] >= min_lim & ...
                [mission.world.victims.t_detected] <= max_lim;
        if ~isempty(find(mask_, 1))
            victim_colors_(mask_, :) = repmat(c_victim_found, length(find(mask_)), 1);
        end
        set(victims, 'CData', victim_colors_);

        % tasks
        task_history_ = history_(history_.action ~= "none", :);
        for i_ = 1:length(robot_id)
            robot_ = mission.robots([mission.robots.id] == robot_id(i_));
            for ii_ = 1:length(actions)
                r_history_ = task_history_(task_history_.robot_id == robot_.id, :);
                entries_ = arrayfun(@(x) split(x, "_"), r_history_.action, 'UniformOutput', false); 
                entries_ = cellfun(@(x) x(1), entries_); 
                if ~isempty(entries_)
                    r_history_ = r_history_(entries_ == actions(ii_), :);
                    xval_ = mission.world.X(str2double([r_history_.node]));
                    yval_ = mission.world.Y(str2double([r_history_.node]));
                else
                    xval_ = nan;
                    yval_ = nan;
                end
                set(task_plots.(robot_.id).(actions(ii_)), ...
                    'XData', xval_, 'YData', yval_);
                xlabel(task_plots.(robot_.id).(actions(ii_)).Parent, ...
                    sprintf('time (%s - %s)', string(min_lim), string(max_lim)));
            end
        end
    end
end

