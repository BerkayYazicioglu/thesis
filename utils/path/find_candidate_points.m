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
    if     (policy == "all_frontier") ... 
        || (policy == "4_frontier") ...
        || (policy == "8_frontier")

        % get all nodes within horizon
        [node_x, node_y] = ind2sub(robot.world.grid_dim, str2double(robot.node));
        p_row = node_x + robot.policy.horizon(:, 1);
        p_col = node_y + robot.policy.horizon(:, 2);
        valid_rows = p_row > 0 & p_row <= robot.world.grid_dim(1);
        valid_cols = p_col > 0 & p_col <= robot.world.grid_dim(2);
        valid = valid_rows & valid_cols;
        p_row = p_row(valid);
        p_col = p_col(valid);
        fov = sub2ind(robot.world.grid_dim, p_row, p_col);
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
        
        if (policy == "all_frontier")
            return
        end
        if (policy == "4_frontier")
            references = [0 pi/2 pi 3*pi/2];
        end
        if (policy == "8_frontier")
            references = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4];
        end

        idx = cellfun(@str2double, points);
        pos = [robot.world.X(idx) robot.world.Y(idx)] ...
            - [robot.world.X(str2double(robot.node)) ...
               robot.world.Y(str2double(robot.node))];
        angles = atan2(pos(:, 2), pos(:, 1));
        % Calculate the distance between the angles
        distance = abs(angles - references);
        % Make sure the distance is between 0 and 2*pi
        distance = mod(distance, 2*pi);
        % find min distance along reach column
        minimums = min(distance);
        l = vecnorm(pos');
        candidates = zeros(4,1);
        for i = 1:length(references)
            candidate = find(distance(:, i) == minimums(i));
            % find the furthest candidate from the robot position
            [~, j] = max(l(candidate));
            candidates(i) = candidate(j);
        end
        candidates(ismember(candidates, robot.node)) = [];
        points = points(candidates);
            
    end
end