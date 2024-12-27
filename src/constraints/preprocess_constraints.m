function output = preprocess_constraints(robot, task)
%CHARGE_CONSTRAINTS Get the minimum time and energy thresholds to perform the
% given task

output = timetable(duration.empty(0,1), [], 'VariableNames', {'energy'});
charger_nodes = [robot.mission.charger.node;
                 robot.mission.charger.schedule.node];
charger_times = [robot.mission.charger.schedule.Time;
                 seconds(inf)];

for i = 1:length(charger_nodes)
    node = charger_nodes(i);
    [~, d] = robot.map.shortestpath(task.node, node);
    min_e = robot.crit_energy + d * robot.energy_per_m;
    max_t = charger_times(i) - seconds(d/robot.speed);
    output(max_t, :) = {min_e};
end
end

