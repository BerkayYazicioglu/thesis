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

%% Multiple run parameters
 
q_inits = {{1,1}, ...
           {50,1}, ...
           {100,1}, ...
           {100,50}, ...
           {100,100}, ...
           {50,100}, ...
           {1,100}, ...
           {1,50}};

repetitions = 3;
experiment_name = "dataset_4";


%% Get config
if exist("results/" + experiment_name, 'dir')
    in = input('directory already exists, continue? [y/n]', 's');
    if lower(in) ~= 'y'
        error('make sure directories are not conflicting');
    end
end

settings = yaml.loadFile('config.yaml');
sim_obj = simulation;

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


%% Perform and save simulations

for i = 1:length(q_inits)
    settings.mission.q_init = q_inits{i};
    for ii = 1:repetitions
        disp('experiment: ' + string(ii+repetitions*(i-1)));

        world = World(settings.world);
        mission = Mission(settings.mission, world);
        gui = Gui(settings.gui, sim_obj, mission, world);
        
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
        
        % Save results
        result_dir = "results/" + experiment_name + '/' + string(ii+repetitions*(i-1));
        mkdir(result_dir);
        
        gui.export_figures(result_dir);
        save(result_dir + "/mission.mat", "mission"); 
        % save logs
        log = gui.sim.logger.Value;
        save(result_dir + "/log.mat", 'log');
    end
end