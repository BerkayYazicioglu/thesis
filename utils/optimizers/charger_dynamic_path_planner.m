%% Center of mass of remaining tasks path planner for charger
function schedule = charger_dynamic_path_planner(charger, time, map, tasks)

    % find the unweighted center of mass of tasks
    locs = reshape([tasks.items.values.location], [2, tasks.items.numEntries()])';
    goal = [mean(locs(:,1)) mean(locs(:,2))];
    % remove the infeasable edges
    c = find(~charger.traversability(map.Edges.EndNodes));
    map = map.rmedge(map.Edges.EndNodes(c,1), ...
                     map.Edges.EndNodes(c,2));
    map = map.rmnode(find(map.Nodes.uncertainty >= 0.1));
    % get only the accessible nodes
    [bin, ~] = conncomp(map);
    candidates = map.Nodes.Name(bin == bin(map.findnode(charger.node)));
    candidates([cellfun(@(x) strcmp(x, charger.node), candidates)]) = [];
    % find the candidate closest to the goal
    candidate_locs = [charger.world.X([cellfun(@(x) str2double(x), candidates)])'; 
                      charger.world.Y([cellfun(@(x) str2double(x), candidates)])']';
    distances = vecnorm((goal - candidate_locs)');
    [min_dist, idx] = min(distances);
    candidate_goal = candidates{idx};
    [p, ~, edges] = map.shortestpath(charger.node, candidate_goal);
    % update schedule
    if height(charger.schedule)
        t_start = max(charger.schedule.Time(end), time);
    else
        t_start = time;
    end
    dt = [arrayfun(@(x) charger.world.environment.Edges(x,:).Weight / charger.speed, edges)];
    Time = seconds([arrayfun(@(x) sum(dt(1:x)), 1:length(dt))]) + t_start;
    p(1) = [];
    max_length = min(charger.control_horizon, length(p));
    Time = Time(1:max_length);
    p = p(1:max_length);
    schedule = timetable(Time(:), [cellfun(@(x) str2num(x), p)]', 'VariableNames', {'node'});
end