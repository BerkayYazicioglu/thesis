%%
clc
clear
close

%% Get the environment data
world_params.data_file = "dummy.json";
world_params.distance_mode = "3d";

world = World(world_params);

%% Robot parameters
r1_params.id = 'ugv_1';
r1_params.cam_range = 50; % m
r1_params.perc_range = 10; % m
r1_params.velocity = 3; % m/s
r1_params.max_step = 1; % m
r1_params.height = 1; % m
r1_params.PI_model_file = "PI_model.fis";
r1_params.color = 'black';


r2_params.id = 'uav_1';
r2_params.cam_range = 75; % m
r2_params.perc_range = 20; % m
r2_params.velocity = 3; % m/s
r2_params.max_step = 100; % m
r2_params.height = 10; % m
r2_params.PI_model_file = "PI_model.fis";
r2_params.color = 'white';

%% simulation
r1 = Robot(r1_params, world);
r2 = Robot(r2_params, world);

r1.place([90 10]);
r2.place([10 10]);


% WIP
v = VideoWriter('simulation');
open(v);

hold(gca, 'on');
world.plot(gca);
r1.plot(gca);
r2.plot(gca);
drawnow

for i = 1:50
    disp(i);

    r1.path_planner();
    for j = 2:4
        r1.move(r1.path(j));
        r1.plot(gca);
        drawnow
    end

    r2.path_planner();
    for j = 2:4
        r2.move(r2.path(j));
        r2.plot(gca);
        drawnow
    end

    im = frame2im(getframe(gcf));
    writeVideo(v,im);
end

close(v);
