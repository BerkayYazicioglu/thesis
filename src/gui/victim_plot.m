function handles = victim_plot(gui, handles)
    if nargin > 1
        % update plot
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            set(handles.(robot.id), 'XData', robot.history.Time, ...
                                    'YData', robot.history.detected_victims);
        end
    else
        % init plot
        hold(gui.plot_axes, 'on');
        
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            handles.(robot.id) = plot(gui.plot_axes, ...
                                      robot.history.Time, ...
                                      robot.history.detected_victims);
        end
        legend(gui.plot_axes, gui.mission.robots.id);
        grid(gui.plot_axes, "on");
        xtickformat(gui.plot_axes, 'hh:mm:ss');
        xlabel(gui.plot_axes, 'timestamp');
        ylabel(gui.plot_axes, 'victim count');
        title(gui.plot_axes, 'Number of unique victim detection by robots');


        hold(gui.plot_axes, 'off');
    end
end

