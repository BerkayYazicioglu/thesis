%% Find candidate frontier points for path generation

function points = find_candidate_points(robot, policy)
    arguments
        robot Robot
        policy string = "all_frontier"
    end
    points = {};

    % find the connected component that the robot is currently in
    % (the map might be disjoint)
    bins = robot.map.conncomp();
    robot_bin = bins(robot.map.findnode(robot.node));
    idx = find(bins == robot_bin);
    map = robot.map.subgraph(robot.map.Nodes.Name(idx));

    if map.numnodes <= 1
        error("Insufficient number of nodes")
    end

    % find all points on the edges of reachable FoV area
    if policy == "all_frontier"
        % get all nodes within FoV
        [fov, ~] = robot.visible_sensor.get_all(str2double(robot.node));
        fov = arrayfun(@num2str, fov, 'UniformOutput', 0);
        % get only the nodes that are in the map 
        fov = fov(ismember(fov, map.Nodes.Name));
        if length(fov) <= 1
            error("Insufficient number of nodes")
        end
        % overlay the FoV on the grid and find the circumference points
        map = robot.world.environment.subgraph(fov);
        degree = map.degree(fov);
        % nodes with less than 8 connections should be at the edge
        points = fov(degree < 8);
    end
end