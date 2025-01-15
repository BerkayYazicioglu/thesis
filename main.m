%%
if exist('sim_obj', 'Var')
    sim_obj.delete;
end
clc
clear 
close

addpath("src")
addpath("src/utils")
addpath("src/constraints")
addpath("src/task_allocators")
addpath("src/gui")
addpath("src/charger_controllers/")

%% Get config
settings = yaml.loadFile('config.yaml');
% create individual robot entries
types = fieldnames(settings.mission.robots);
for r = 1:length(types)
    type = types{r};
    count = settings.mission.robots.(type).count;
    for i = 1:count
        r_params = yaml.loadFile(type+".yaml");
        robots.(type+"_"+num2str(i)) = r_params;
    end
end
settings.mission.robots = robots;

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
        world = World(settings.world);
        mission = Mission(settings.mission, world);
        gui.world = world;
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
                      {'1'});
result_dir = "results/simulation/" + result_dir{1};
mkdir(result_dir);

gui.export_figures(result_dir);
save(result_dir + "/mission.mat", "mission"); 
% save logs
log = gui.sim.logger.Value;
save(result_dir + "/log.mat", 'log');
