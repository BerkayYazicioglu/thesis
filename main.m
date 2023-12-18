%%
clc
clearvars -except gui
close

%% Get config
settings = yaml.loadFile('config.yaml');

if ~exist('gui', 'Var')
    gui = simulation;
elseif ~isvalid(gui)
    gui = simulation;
end
cla(gui.world,'reset')
cla(gui.population,'reset')

hold(gui.world,'on')
hold(gui.population,'on')

%% Generate the mission
mission = Mission(settings);

mission.plot(gui);

while mission.t <= mission.t_end 
%    try 
        waitfor(gui.start, 'Value', 'On'); 
        mission.run();
        mission.plot();
        drawnow
%    catch 
%        break; 
%    end
end

%% simulation

% WIP
% v = VideoWriter('simulation');
% open(v);
% 
% hold(gca, 'on');
% world.plot(gca);
% r1.plot(gca);
% r2.plot(gca);
% drawnow
% 
% for i = 1:50
%     disp(i);
% 
%     r1.path_planner();
%     for j = 2:4
%         r1.move(r1.path(j));
%         r1.plot(gca);
%         drawnow
%     end
% 
%     r2.path_planner();
%     for j = 2:4
%         r2.move(r2.path(j));
%         r2.plot(gca);
%         drawnow
%     end
% 
%     im = frame2im(getframe(gcf));
%     writeVideo(v,im);
% end
% 
% close(v);
