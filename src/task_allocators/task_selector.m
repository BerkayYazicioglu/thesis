function [pp, task_idx] = task_selector(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

% ============================== params ===================================
max_work_limit = 5;
% =========================================================================

if isempty(preprocessing.tasks)
    pp = preprocessing;
    task_idx = 1:length(pp.tasks);
    return
end

% calculate all distance and travel time pairs
all_nodes = [robot.node preprocessing.tasks.node];
D = distance_matrix(robot, all_nodes, 1);
T = seconds(D./robot.speed) + [seconds(0) preprocessing.dt];

% add task types to preprocessing
preprocessing.outcomes.task_types = arrayfun(@(x) preprocessing.tasks(x).type, preprocessing.outcomes.task_idx);
preprocessing.outcomes.values = zeros(height(preprocessing.outcomes), 1);

% process sets
sets = groupcounts(preprocessing.outcomes, ["task_idx" "actions"]);
sets.priority = zeros(height(sets), 1);
sets.norm = zeros(height(sets), 1); 
sets.capability = zeros(height(sets), 1);
flags = false(height(sets), 1);
for i = 1:height(sets)
    task = preprocessing.tasks(sets.task_idx(i));
    if task.type == "map"
        sets.priority(i) = max(0, ...
            numel(robot.mission.world.environment.neighbors(task.node)) - ...
            numel(robot.mission.map.neighbors(task.node)));
        sets.norm(i) = max(sets.GroupCount);
        sets.capability(i) = robot.mapper.capability;
        flags(i) = true;
    else
        sets.priority(i) = task.priority;
        sets.norm(i) = max(sets.GroupCount);
        sets.capability(i) = robot.detector.capability;
    end
end
if sum(flags)
    sets.priority(flags) = sets.priority(flags) / (max(sets.priority(flags) + 0.0001));
end

% process elements
elements = sparse(height(sets), height(preprocessing.outcomes));
values = zeros(height(sets), 1);
for i = 1:height(sets)
    elements(i,:) = preprocessing.outcomes.task_idx == sets.task_idx(i) & ...
                    preprocessing.outcomes.actions == sets.actions(i);


    action_eval = evalfis(robot.task_eval, ...
        [sets.capability(x(n)) ...
        sum(additions) / sets.norm(x(n)) ...
        1 - median(preprocessing.outcomes.distances(additions)) / max(preprocessing.outcomes.distances(additions)), ...
        sets.priority(x(n))]);
end



end