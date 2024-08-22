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
save_file_name = settings.world.save_file_name;
save_dir = "results/" + save_file_name;
mkdir(save_dir);

world = World(settings.world);
mission = Mission(settings.mission, world);
plots = axes();
gui = Gui(settings.mission.gui, sim_obj, mission, plots);
gui.run();

if settings.world.capture_video
    v = VideoWriter(save_dir + "/video");
    open(v);
end
while mission.schedule.Time(end) <= mission.t_end 
%    try 
        if gui.restart_flag
            mission = Mission(mission.settings, world);
            gui = Gui(settings.mission.gui, sim_obj, mission, plots);
            gui.run();
        end
        waitfor(sim_obj.start, 'Value', 'On'); 

        if settings.world.capture_video
            im = frame2im(getframe(gui.gui.UIFigure));
            writeVideo(v,im);
        end
      
        mission.run();
        gui.run();
%    catch 
%        break; 
%    end
end

%% save simulation run
mission_log = gui.gui.logger.Value;
save(save_dir + "/mission.mat", "mission");
save(save_dir + "/log.mat", "mission_log");

if settings.world.capture_video
    close(v);
end












