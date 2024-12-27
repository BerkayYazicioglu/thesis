function handles = area_plot(gui, handles)
    if nargin > 1
        % update plot
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            set(handles.(robot.id), 'XData', robot.history.Time, ...
                                    'YData', robot.history.mapped_area);
        end
    else
        % init plot
        hold(gui.plot_axes, 'on');
        
        for r = 1:length(gui.mission.robots)
            robot = gui.mission.robots(r);
            handles.(robot.id) = plot(gui.plot_axes, ...
                                      robot.history.Time, ...
                                      robot.history.mapped_area);
        end
        legend(gui.plot_axes, gui.mission.robots.id);
        grid(gui.plot_axes, "on");
        xtickformat(gui.plot_axes, 'hh:mm:ss');
        xlabel('timestamp');
        ylabel('number of locations');
        title('Total area mapped by robots');

        hold(gui.plot_axes, 'off');
    end
end