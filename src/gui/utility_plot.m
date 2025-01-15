function handles = utility_plot(gui, handles)
    if nargin > 1
        % update plot
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            r_table = gui.mission.history.pp(gui.mission.history.pp.robot == robot.id, :);
            set(handles.(robot.id), ...
                'XData', r_table.Time, ...
                'YData', r_table.utility);
        end
    else
        % init plot
        hold(gui.plot_axes, 'on');
        
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            r_table = gui.mission.history.pp(gui.mission.history.pp.robot == robot.id, :);
            handles.(robot.id) = scatter(gui.plot_axes, ...
                 r_table.Time, r_table.utility, 'filled');
        end
        legend(gui.plot_axes, gui.mission.robots.id);
        grid(gui.plot_axes, "on");
        xtickformat(gui.plot_axes, 'hh:mm:ss');
        xlabel(gui.plot_axes, 'timestamp');
        ylabel(gui.plot_axes, 'utility value');
        title(gui.plot_axes, 'Utility values of the task allocations of robots');

        hold(gui.plot_axes, 'off');
    end
end