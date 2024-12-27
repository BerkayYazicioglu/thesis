function return_schedule = generate_return_path(robot)
%GENERATE_RETURN_PATH Constructs a return path to the charger
return_schedule = timetable();

charger_nodes = [robot.mission.charger.node;
                 robot.mission.charger.schedule.node];
charger_times = [robot.mission.charger.schedule.Time;
                 seconds(inf)];
% generate paths to all charger nodes and calculate times
for i = 1:length(charger_nodes)
    [p, d, edges] = robot.map.shortestpath(robot.node, charger_nodes(i));
    t = robot.time; 
    for j = 2:length(p)
        t(j) = t(j-1) + seconds(robot.map.Edges.Weight(edges(j-1))/robot.speed);
    end
    % check if the robot makes it in time
    if t(end) <= charger_times(i)
        e = robot.energy - d * robot.energy_per_m;
        if e <= robot.crit_energy
            break
        end
        % the path is valid
        actions = repmat("none", 1, length(p));
        actions(end) = "charge";
        % construct the timetable
        return_schedule = timetable(t(:)-robot.time, p(:), actions(:), 'VariableNames', {'node', 'action'});
        return
    end  
end
error("Robot can not return safely to the charger.");
   
end

