function multi_victim_analysis(missions, gui, ~)

    % ========= params ==========
    x_label_interval = 30 * seconds(60); % minutes
    markers = dictionary("detected", "pentagram", "revisited", ".");
    % ===========================
    
    panel = gui.RightPanel;
    gui.single_plot_options.Items = "none";
    gui.single_plot_options.Value = "none";
    
    experiments = fields(missions);
    colors = distinguishable_colors(length(experiments), 'white');
    health = {};
    count = {};

    for j = 1:length(experiments)
        M = missions.(experiments{j});

        count_data = timetable(seconds(0), 0, 'VariableNames', {'count'});
        health_data = timetable(duration.empty(0,1), [], 'VariableNames', {'health'});
        for k = 1:length(M)
            mission = M(k);
            % go over victims and construct data
            victim_data = timetable();
            for i = 1:length(mission.world.victims)
                victim = mission.world.victims(i);
                if ~isempty(victim.history)
                    history = victim.history;
                    history.victim_id = i * ones(height(history), 1);
                    victim_data = [victim_data; history];
                end
            end
            victim_data = sortrows(victim_data, 'Time');
            detected_data = victim_data(victim_data.status == 'detected', :);
            detected_data.count = [1:height(detected_data)]';
            
            count_data = synchronize(count_data, detected_data(:, "count"), 'union');
            health_data = [health_data; detected_data(:, "health")];
        end
        count_data(:, 1) = [];
        count_data(1, :) = array2timetable(zeros(1, size(count_data, 2)), 'RowTimes', seconds(0));
        count_data = fillmissing(count_data, 'previous'); 
        count_data.mean = mean(count_data{:, :}, 2);
        count_data.max = max(count_data{:, :}, [], 2);
        count_data.min = min(count_data{:, :}, [], 2);
        health_data = sortrows(health_data, 'Time');
        count{end+1} = count_data(:, {'mean', 'max', 'min'});
        health{end+1} = health_data;
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

