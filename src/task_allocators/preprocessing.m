function output = preprocessing(robot)
%PREPROCESSING Find the closest (Euclidean) tasks the robot is able to perform
%              to the given node using shortest path distance on the map 
t0 = tic;

num_tasks = robot.policy.num_tasks;
output.outcomes = table();
output.tasks = Task.empty;
output.dt = duration.empty;
output.de = [];
output.constraints = {};

% get the accessible nodes 
[bin, ~] = conncomp(robot.map);
accessible = 0 * bin;
robot_bin = bin(robot.map.findnode(robot.node));
accessible = accessible | (bin == robot_bin);
accessible = robot.map.Nodes.Name(accessible);
tasks = robot.mission.tasks(ismember([robot.mission.tasks.node], accessible));
tasks = tasks([arrayfun(@(t) ismember(robot.id, t.R_k), tasks)]);
if isempty(tasks)
    disp(robot.id + " | preprocessing (no valid tasks) | " + toc(t0));
    return;
end

if robot.policy.preprocessing == "closest"
    task_distances = vecnorm(robot.world.get_coordinates(robot.node) - ...
                             robot.world.get_coordinates([tasks.node]), 2, 2);
elseif robot.policy.preprocessing == "shortest_path"
    task_distances = distance_matrix(robot, [tasks.node] , 1);
end

[~, idx] = sort(task_distances);
tasks = tasks(idx);

% poll through tasks, predict and populate num_tasks amount of valid
% tasks for preprocessing
idx = 1;
for i = 1:length(tasks)
    [outcomes, de, dt] = tasks(i).predict(robot);
    if ~isempty(outcomes)
        outcomes.task_idx = i * ones(height(outcomes),1);
        output.tasks(idx) = tasks(i);
        output.dt(idx) = dt;
        output.de(idx) = de;
        output.outcomes = [output.outcomes; outcomes];
        output.constraints{idx} = preprocess_constraints(robot, tasks(i));
        idx = idx + 1;
    end
    if idx > num_tasks
        break;
    end
end

disp(robot.id + " | preprocessing | " + toc(t0));
end

