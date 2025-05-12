function [conflicts, schedules] = detect_conflicts(mission)
%DETECT_CONFLICTS Detect conflicts between all robots
%
% conflicts -> timetable for conflicting pairs, and conflict regions

% combine the actions per schedule 
schedules = timetable();
r_count = 0;
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

    if r_count > 1
        node = schedule.node(2);
        coord_area = mission.world.environment.nearest(node, mission.settings.coordination_radius, 'Method', 'unweighted');
        coord_area = [coord_area; node];
        % hardcoded for two robots
        other_node = schedules.node(2);
        if ~ismember(other_node, coord_area)
            continue;
        end
    end
    schedules = [schedules; schedule];
    r_count = r_count + 1;
end

remove_idx = [];
for i = 1:height(schedules)
    if schedules.action(i) == "none"
        continue;
    end
    task_flags = schedules.node(i) == [mission.tasks.node] ...
               & schedules.type(i) == [mission.tasks.type];
    task_idx = find(task_flags);
    if isempty(task_idx)
        remove_idx = [remove_idx i];
        continue;
    end
end
schedules(remove_idx, :) = [];

if r_count <= 1
    schedules = timetable();
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
        mission.settings.conflict_radius, "Method", "unweighted");
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

