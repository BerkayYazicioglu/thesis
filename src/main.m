%%
if exist('sim_obj', 'Var')
    sim_obj.delete;
end
clc
clear 
close

addpath("utils")
addpath("constraints")
addpath("task_allocators")
addpath("gui")

%% Get config
settings = yaml.loadFile('../config.yaml');
% create individual robot entries
types = fieldnames(settings.mission.robots);
for r = 1:length(types)
    type = types{r};
    count = settings.mission.robots.(type).count;
    for i = 1:count
        r_params = rmfield(settings.mission.robots.(type), 'count');
        r_params.q_init = settings.mission.robots.(type).q_init{i};
        robots.(type+"_"+num2str(i)) = r_params;
    end
end
settings.mission.robots = robots;

% load models
settings.mission.mcdm = load(settings.mission.mcdm).MCDM;

%% Generate the simulation objects
sim_obj = simulation;
world = World(settings.world);
mission = Mission(settings.mission, world);
gui = Gui(settings.gui, sim_obj, mission, world);

%% Start simulation
t_total = 0;
while mission.time <= mission.t_end && ~mission.all_idle
    % check if the simulation needs to be restarted
    if gui.restart_flag
        t_total = 0;
        mission = Mission(gui.param_cache, world);
        gui.init(mission);
        disp('simulation restarted');
    end
    waitfor(sim_obj.start, 'Value', 'On'); 

    t0 = tic;

    mission.run();
    gui.run();
     
    t_total = t_total + toc(t0);
end
disp("Total simulation time: " + string(t_total) + " s");

%% Save results
result_dir = inputdlg({'Enter save folder name'}, ...
                      'Save simulation results', ...
                      1, ...
                      {'simulation'});
result_dir = "../results/" + result_dir{1};
mkdir(result_dir);

gui.export_figures(result_dir);
save(result_dir + "/mission.mat", "mission"); 