function handles = tasks_plot(gui, handles)
    if nargin > 1
        % update plot
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            r_table = gui.mission.history.tasks(gui.mission.history.tasks.robot == robot.id, :);
            set(handles.(robot.id).completed, ...
                'XData', r_table.Time, ...
                'YData', cellfun(@(x) numel(x), r_table.completed_nodes));
            set(handles.(robot.id).spawned, ...
                'XData', r_table.Time, ...
                'YData', cellfun(@(x) numel(x), r_table.spawned_nodes));
        end
    else
        % init plot
        hold(gui.plot_axes, 'on');
        
        c = distinguishable_colors(length(gui.mission.robots));
        legend_entries = {};
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            r_table = gui.mission.history.tasks(gui.mission.history.tasks.robot == robot.id, :);
            handles.(robot.id).completed = stem(gui.plot_axes, ...
                 r_table.Time, cellfun(@(x) numel(x), r_table.completed_nodes), ...
                'MarkerFaceColor','red',...
                'Color', c(r,:));
            handles.(robot.id).spawned = stem(gui.plot_axes, ...
                 r_table.Time, cellfun(@(x) numel(x), r_table.spawned_nodes), ...
                'MarkerFaceColor','green',...
                'Color', c(r,:));

            % plots for legend
            legend_entries{r} = plot(nan, 'Color', c(r,:));
        end
        legend_entries{end+1} = scatter(nan,nan,'filled','green');
        legend_entries{end+1} = scatter(nan,nan,'filled','red');
        legend(gui.plot_axes, [legend_entries{:}], [gui.mission.robots.id "spawned" "completed"]);
        grid(gui.plot_axes, "on");
        xtickformat(gui.plot_axes, 'hh:mm:ss');
        xlabel('timestamp');
        ylabel('task count');
        title('Number of spawned and completed tasks of robots');


        hold(gui.plot_axes, 'off');
    end
end