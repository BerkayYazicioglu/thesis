%%
clc
clearvars -except sim_obj
close
addpath('utils')
addpath('utils/optimizers')
addpath('utils/path')

%% Get config
settings = yaml.loadFile('config.yaml');

if ~exist('sim_obj', 'Var')
    sim_obj = simulation;
elseif ~isvalid(sim_obj)
    sim_obj = simulation;
end

%% Generate the mission

world = World(settings.world);
mission = Mission(settings.mission, world);
gui = Gui(settings.mission.gui, sim_obj, mission);
gui.run();

while mission.t <= mission.t_end 
%    try 
        waitfor(sim_obj.start, 'Value', 'On'); 
        mission.run();
        gui.run();
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









