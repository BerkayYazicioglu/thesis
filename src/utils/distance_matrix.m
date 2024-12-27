function D = distance_matrix(robot, nodes, n_dim)
%DISTANCE_MATRIX calculate the all pairwise distances between given nodes

[unique_nodes, ~, a_idx] = unique(nodes, 'stable');
if n_dim == 1
    D = robot.map.distances(robot.node, unique_nodes);
elseif n_dim == 2
    D = robot.map.distances(unique_nodes, unique_nodes);
end

% update to include the repeated nodes
for i = 2:length(a_idx)
    idx = find(a_idx(i) == a_idx(1:i-1), 1);
    if ~isempty(idx)
        % add column
        D = [D(:,1:i-1) D(:,idx) D(:,i:end)];
        % add row
        if n_dim == 2
            D = [D(1:i-1,:); D(idx,:); D(i:end,:)];
        end
    end
end

end

