function output = center_of_mass(charger)
%CENTER_OF_MASS Controller for charger robot to steer towards to the valid
% center of mass of remaining tasks

goal_passivity = 3;

c = find(~charger.traversability(charger.mission.map.Edges.EndNodes));
map = charger.mission.map.rmedge(charger.mission.map.Edges.EndNodes(c,1), ...
                             charger.mission.map.Edges.EndNodes(c,2));
map = map.rmnode(map.Nodes.Name(~map.Nodes.visible));
% get only the accessible nodes
[bin, ~] = conncomp(map);
candidates = map.Nodes.Name(bin == bin(map.findnode(charger.node)));
% get all candidates that are within task proximity range
valid_tasks = false(size(charger.mission.tasks));
for r = 1:length(charger.mission.robots)
    robot = charger.mission.robots(r);
    reachable = ismember([charger.mission.tasks.node], robot.map.Nodes.Name);
    attemptable = arrayfun(@(x) ismember(robot.id, x.R_k), charger.mission.tasks);
    valid_tasks = valid_tasks | (attemptable & reachable);
end
task_nodes = unique([charger.mission.tasks(valid_tasks).node]);
if isempty(task_nodes)
    charger.schedule = timetable(seconds(inf), charger.node, 'VariableNames', {'node'});
    output.candidates = string.empty;
    output.schedule = charger.schedule;
    output.goal = string.empty;
    return
end
valid_area = [arrayfun(@(x) charger.mission.map.nearest(x, ...
    charger.policy.task_proximity, ...
    'Method', 'unweighted')', task_nodes, ...
    'UniformOutput', false)];
valid_area = unique([valid_area{:}]);
valid_area = setdiff(valid_area, task_nodes);
if isempty(valid_area)
    valid_area = charger.node;
end
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
% apply goal passivity
if ismember(candidate_goal, map.nearest(charger.node, goal_passivity, 'Method', 'unweighted')) && ...
   ismember(charger.node, candidates)
    candidate_goal = charger.node;
end
[p, ~, edges] = map.shortestpath(charger.node, candidate_goal);

% make sure the robots can always return safely from
% scheduled positions
return_flags = arrayfun(@(x) ~isempty(x.return_schedule), charger.mission.robots);
if any(~return_flags)
    % at least one robot has an empty return schedule which means it is
    % either docked or on its way
    robots = charger.mission.robots(~return_flags);
    otw_flags = arrayfun(@(x) x.schedule.action(end) == "charge", robots);
    if any(otw_flags)
        % if any robots are on the way, do not move until the robot reaches
        % the charger
        p = charger.node;
        t = max(arrayfun(@(x) x.schedule.Time(end), robots(otw_flags)));
        charger.schedule(t,:) = {p}; 

        output.candidates = candidates;
        output.schedule = charger.schedule;
        output.goal = candidate_goal;
        output.p = p;
        output.t = t;
        return
    end
end
if all(~return_flags)
    % all robots are docked, the end of the path time should be at
    % least the minimum of robot schedules
    robot_times = arrayfun(@(x) x.schedule.Time(1), charger.mission.robots);
    if length(p) > 1
        dt = [arrayfun(@(x) charger.world.environment.Edges(x,:).Weight / charger.speed, edges)];
        t = seconds([arrayfun(@(x) sum(dt(1:x)), 1:length(dt))]) + charger.time;
        p(1) = [];
        p = p(1: min(length(p), charger.policy.control_horizon));
        t = t(1: min(length(t), charger.policy.control_horizon));
        t(end) = max([t(end) min(robot_times)]);
    else
        t = max([charger.time max(robot_times)]);
    end
elseif length(p) == 1
     t = max([charger.time max(arrayfun(@(x) x.schedule.Time(end), charger.mission.robots))]);
else
    % there are some robots that may return to the charger
    p(1) = [];
    p = p(1: min(length(p), charger.policy.control_horizon));

    return_flags = find(return_flags);
    new_schedules = cell(size(return_flags));
    for i = 1:length(return_flags)
        robot = charger.mission.robots(return_flags(i));
        robot.update_maps();
        robot_e = robot.return_schedule.energy(1);
        robot_node = robot.return_schedule.node(1);
        robot_t = robot.return_schedule.Time(1);
        for ii = 1:length(p)
            % check if it is possible for robot to return to the path
            new_schedules{i} = generate_return_path(robot, robot_node, robot_t, robot_e, p(end-ii+1), seconds(inf), false);
            if ~isempty(new_schedules{i})
                break
            end
            if ii == length(p)
                new_schedules{i} = generate_return_path(robot, robot_node, robot_t, robot_e, charger.node, seconds(inf), true);
            end
        end
    end
    dt = [arrayfun(@(x) charger.world.environment.Edges(x,:).Weight / charger.speed, edges)];
    t = seconds([arrayfun(@(x) sum(dt(1:x)), 1:length(dt))]) + charger.time;
    t = t(1: min(length(t), charger.policy.control_horizon));

    p = [charger.node p];
    for i = 1:length(t)
        flags = cellfun(@(x) x.node(end) == p(i), new_schedules);
        if ~any(flags)
            continue;
        end
        dt_ = max([t(i) cellfun(@(x) x.Time(end), new_schedules(flags))]) - t(i);
        t(i:end) = t(i:end) + dt_;
        flags = find(flags);
        for ii = 1:length(flags)
            new_schedules{flags(ii)}.Time(end) = t(i);
        end
    end
    % distribute the new return schedules
    for i = 1:length(return_flags)
        charger.mission.robots(return_flags(i)).return_schedule = new_schedules{i};
    end
    p(1) = [];
end

% construct the schedule
for i = 1:length(p)
    charger.schedule(t(i),:) = {p(i)}; 
end

output.candidates = candidates;
output.schedule = charger.schedule;
output.goal = candidate_goal;
output.p = p;
output.t = t;

end

