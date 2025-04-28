function [conflicts, schedules] = detect_conflicts(mission)
%DETECT_CONFLICTS Detect conflicts between all robots
%
% conflicts -> timetable for conflicting pairs, and conflict regions

% combine the actions per schedule 
schedules = timetable();
for r = 1:length(mission.robots)
    robot = mission.robots(r);
    if isempty(robot.cache)
        continue;
    end

    row = robot.cache(robot.cache_idx,:);
    schedule = timetable(seconds(row.t{1}(:)), ...
        row.nodes{1}(:), ...
        row.actions{1}(:), ...
        row.e{1}(:), ...
        row.u_map{1}(:), ...
        row.u_search{1}(:), ...
        'VariableNames', {'node', 'action', 'energy', 'u_map', 'u_search'});
    schedule.robot_idx = repmat(r, height(schedule), 1);
    schedule.type = arrayfun(@(x) extractBefore(x, '_'), schedule.action);
    schedule = [timetable(robot.time, robot.node, "none", robot.energy, 0, 0, r, "none", ...
        'VariableNames', {'node', 'action', 'energy', 'u_map', 'u_search', 'robot_idx', 'type'});
                schedule];
    schedules = [schedules; schedule];
end

% determine conflicts 
conflicts = table([], [], 'VariableNames', {'1', '2'});
for i = 1:height(schedules)-1
    cur_row = schedules(i,:);
    if cur_row.action == "none"
        continue;
    end
    % get the nearest nodes within radius
    nearest_nodes = mission.map.nearest(cur_row.node, ...
        mission.coordination_radius, "Method", "unweighted");
    nearest_nodes = [nearest_nodes; cur_row.node];
    for j = i+1:height(schedules)
        row = schedules(j,:);
        if row.action == "none"
            continue;
        end
        if row.robot_idx == cur_row.robot_idx
            continue
        end
        if string(cur_row.type{1}) == string(row.type{1})
            if ismember(row.node, nearest_nodes)
                conflicts(end+1, :) = {i, j};
            end
        end
    end
end

end

