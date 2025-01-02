function [conflicts, schedules] = detect_conflicts(mission)
%DETECT_CONFLICTS Detect conflicts between all robots
%
% conflicts -> timetable for conflicting pairs, and conflict regions

% combine the actions per schedule 
schedules = timetable();
filtered_actions = ["none" "charge" "charge_done"];
for r = 1:length(mission.robots)
    schedule = mission.robots(r).schedule;
    schedule = schedule(~ismember(schedule.action, filtered_actions), :);
    schedule.robot_idx = repmat(r, height(schedule), 1);
    schedule.type = arrayfun(@(x) extractBefore(x, '_'), schedule.action, ...
        'UniformOutput', false);
    schedules = [schedules; schedule];
end

% determine conflicts 
conflicts = table([], [], 'VariableNames', {'1', '2'});
for i = 1:height(schedules)-1
    cur_row = schedules(i,:);
    % get the nearest nodes within radius
    nearest_nodes = mission.map.nearest(cur_row.node, ...
        mission.coordination_radius, "Method", "unweighted");
    nearest_nodes = [nearest_nodes; cur_row.node];
    for j = i+1:height(schedules)
        row = schedules(j,:);
        if row.robot_idx == cur_row.robot_idx
            continue
        end
        if cur_row.type{1} == row.type{1}
            if ismember(row.node, nearest_nodes)
                conflicts(end+1, :) = {i, j};
            end
        end
    end
end

end

