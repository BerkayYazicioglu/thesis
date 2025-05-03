function output = preprocessing(robot)
%PREPROCESSING Find the closest (Euclidean) tasks the robot is able to perform
%              to the given node using shortest path distance on the map 

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
    return;
end

D = distance_matrix(robot, [tasks.node] , 1);
T = seconds(D./robot.speed) + robot.time;
E = robot.energy - D * robot.energy_per_m;

[~, idx] = sort(D);
tasks = tasks(idx);
T = T(idx);
E = E(idx);

% poll through tasks, predict and populate num_tasks amount of valid
% tasks for preprocessing
idx = 1;
flagged_idx = 1;

if robot.policy.optimizer == "random" 
    num_tasks = 50;
    tasks = tasks(randperm(numel(tasks), min(numel(tasks), num_tasks)));
    num_tasks = numel(tasks);
elseif robot.policy.optimizer == "greedy_t" 
    num_tasks = 50;
    tasks = tasks(1:min(numel(tasks), num_tasks));
    num_tasks = numel(tasks);
end

for i = 1:length(tasks)
    [outcomes, de, dt] = tasks(i).predict(robot);
    if ~isempty(outcomes)
        constraints = preprocess_constraints(robot, tasks(i));
        % check if the constraint is satisfied as the first selection
        if ~check_constraints(constraints, T(i) + dt, E(i) - de)
            continue
        end
        outcomes.task_idx = idx * ones(height(outcomes),1);
        output.tasks(idx) = tasks(i);
        output.dt(idx) = dt;
        output.de(idx) = de;
        output.outcomes = [output.outcomes; outcomes];
        output.constraints{idx} = constraints;
        idx = idx + 1;
        if tasks(i).flag
            flagged_idx = flagged_idx + 1;
        end
    end
    if flagged_idx > num_tasks 
        break;
    end
end

end

