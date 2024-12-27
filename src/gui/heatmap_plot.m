function handles = heatmap_plot(gui, handles)
    if nargin > 1
        % update plot
        c = zeros(size(gui.mission.world.X, 1), ...
                  size(gui.mission.world.X, 2));
        for r = 1:length(gui.mission.robots)
            [GC,GR] = groupcounts(gui.mission.robots(r).history.node);
            GR = str2double(GR);
            c(GR) = c(GR) + GC;
        end
        c = c ./ max(c,[],'all');
        c = flipud(floor(c * 256));
        set(handles.data, 'CData', c);
    else
        % init plot
        handles.world = gui.world.init_gui(gui.plot_axes);
        set(handles.world.terrain, 'FaceAlpha', 0);
        handles.world.destruction.Visible = 'off';

        hold(gui.plot_axes, 'on');

        c = zeros(size(gui.world.X, 1), ...
                  size(gui.world.Y, 2));
        for r = 1:length(gui.mission.robots)
            [GC,GR] = groupcounts(gui.mission.robots(r).history.node);
            GR = str2double(GR);
            c(GR) = c(GR) + GC;
        end
        c = c ./ max(c,[],'all');
        c = flipud(floor(c * 256));
        handles.data = imagesc(gui.plot_axes, ...
                        [gui.world.X(1,1) gui.world.X(1,end)], ...
                        [gui.world.Y(end,1) gui.world.Y(1,1)], ...
                        c);
        handles.data.AlphaData = 0.7;

        hold(gui.plot_axes, 'off');
    end
end

