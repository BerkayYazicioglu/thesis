%% Check for charging constraints given a location and time

function flag = charge_constraints(charger, robot, node, e, t)
    [~, idx] = min(abs(charger.schedule.Time - t));
    charger_node = num2str(charger.schedule.node(idx));
    % find the shortest path to the charger node
    [~, d] = robot.map.shortestpath(node, charger_node);
    e = e - d * robot.energy_per_m;
    flag = e > 3;
end