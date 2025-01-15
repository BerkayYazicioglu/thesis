function return_schedule = generate_return_path(robot, node, time, energy, charger_nodes, charger_times, flag)
%GENERATE_RETURN_PATH Constructs a return path to the charger
return_schedule = timetable();
time.Format = 'hh:mm:ss';

% generate paths to all charger nodes and calculate times
for i = 1:length(charger_nodes)
    [p, ~, edges] = robot.map.shortestpath(node, charger_nodes(i));
    t = time; 
    e = energy;
    for j = 2:length(p)
        d = robot.map.Edges.Weight(edges(j-1));
        t(j) = t(j-1) + seconds(d / robot.speed);
        e(j) = e(j-1) - d * robot.energy_per_m;
    end
    % check if the robot makes it in time
    if t(end) <= charger_times(i)
        if e(end) <= robot.crit_energy
            break
        end
        % the path is valid
        actions = repmat("none", 1, length(p));
        actions(end) = "charge";
        % construct the timetable
        return_schedule = timetable(t(:), p(:), actions(:), e(:), ...
            'VariableNames', {'node', 'action', 'energy'});
        return
    end  
end
if flag
    error("Robot can not return safely to the charger.");
end
end

