function dynamic(charger)
%DYNAMİC Summary of this function goes here
%   Detailed explanation goes here

% remove the infeasable edges
c = find(~charger.traversability(charger.mission.map.Edges.EndNodes));
map = charger.mission.map.rmedge(charger.mission.map.Edges.EndNodes(c,1), ...
                             charger.mission.map.Edges.EndNodes(c,2));
map = map.rmnode(map.Nodes.Name(~map.Nodes.visible));
% get only the accessible nodes
[bin, ~] = conncomp(map);
candidates = map.Nodes.Name(bin == bin(map.findnode(charger.node)));
% get all candidates that are within task proximity range
valid_tasks = true(size(charger.mission.tasks));
for r = 1:length(charger.mission.robots)
    robot = charger.mission.robots(r);
    reachable = ismember([charger.mission.tasks.node], robot.map.Nodes.Name);
    attemptable = arrayfun(@(x) ismember(robot.id, x.R_k), charger.mission.tasks);
    valid_tasks = valid_tasks | (attemptable & reachable);
end
task_nodes = unique([charger.mission.tasks(valid_tasks).node]);
if isempty(task_nodes)
    charger.schedule = timetable(seconds(inf), charger.node, 'VariableNames', {'node'});
    return
end
valid_area = [arrayfun(@(x) charger.mission.map.nearest(x, ...
    charger.policy.task_proximity, ...
    'Method', 'unweighted')', task_nodes, ...
    'UniformOutput', false)];
valid_area = unique([valid_area{:}]);
valid_area = setdiff(valid_area, task_nodes);
valid_area = [valid_area charger.node];
% filter candidates that match the valid area
candidates = candidates(ismember(candidates, valid_area));

% find the unweighted center of mass of tasks
locs = charger.world.get_coordinates(task_nodes);
com = [mean(locs(:,1)) mean(locs(:,2))];
% find the candidate closest to the goal
candidate_locs = charger.world.get_coordinates(candidates);
distances = vecnorm((com - candidate_locs)');
[~, idx] = min(distances);
candidate_goal = candidates(idx);
[p, ~, edges] = map.shortestpath(charger.node, candidate_goal);

% make sure the robots can always return safely from
% scheduled positions

robot_max_t = charger.time;
for r = 1:length(charger.mission.robots)
    robot =  charger.mission.robots(r);
    % check if the robot is docked
    if isempty(robot.return_schedule)
        robot_max_t(end+1) = robot.schedule.Time(1);
    else
        robot_max_t(end+1) = robot.return_schedule.Time(end);
    end
end
t_start = max(robot_max_t);
% generate the schedule
if length(p) > 1
    dt = [arrayfun(@(x) charger.world.environment.Edges(x,:).Weight / charger.speed, edges)];
    t = seconds([arrayfun(@(x) sum(dt(1:x)), 1:length(dt))]) + t_start;
    p(1) = [];
else
    t = t_start;
end

for i = 1:min(charger.policy.control_horizon, length(t))
    charger.schedule(t(i),:) = {p(i)}; 
end


end

