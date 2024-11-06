%% Heuristic escape path from a local minima resulted by a lack of nearby tasks
% assuming path{1} ~= robot.node

function path = local_minima_heuristic(robot)
    norm_range = [0.1 1];

    % find the unweighted center of mass of tasks
    locs = reshape([robot.policy.tasks.items.values.location], ...
                   [2, robot.policy.tasks.items.numEntries()])';
    robot_loc = [robot.world.X(str2num(robot.node)) 
                 robot.world.Y(str2num(robot.node))]';
    distances = vecnorm(locs' - robot_loc');
    W = 1 - normalize(distances, 'range', norm_range);

    meanW = mean(W(:));
    goal = [mean(W(:) .* locs(:,1)) / meanW
            mean(W(:) .* locs(:,2)) / meanW]';

    % find the candidate closest to the goal
    [bin, ~] = conncomp(robot.map);
    candidate_nodes = robot.map.Nodes.Name(bin == bin(robot.map.findnode(robot.node)));
    candidate_locs = [robot.world.X([cellfun(@(x) str2double(x), candidate_nodes)])'; 
                      robot.world.Y([cellfun(@(x) str2double(x), candidate_nodes)])']';
    distances = vecnorm((goal - candidate_locs)');
    [~, idx] = min(distances);
    candidate_goal = robot.map.Nodes.Name(idx);
    
    [p, ~, ~] = robot.map.shortestpath(robot.node, candidate_goal);
    path = p(2:robot.policy.configs.prediction_horizon+1);

    % plot
    if robot.world.params.detail_plots
        figure;
        P = [locs(:,1) locs(:,2) W'];
        P = unique(P,'rows');
        hold on
        plot3(P(:,1),P(:,2),P(:,3),'.');
        stem3(goal(1), goal(2),  max(W), 'o', "filled");
        stem3(robot_loc(1), robot_loc(2),  max(W), 'd', "filled");
        stem3(robot.world.X([cellfun(@(x) str2double(x), p)]), ...
              robot.world.Y([cellfun(@(x) str2double(x), p)]), ...
              max(W) * ones(size(p)), '.', "filled");
        hold off
    end
end
