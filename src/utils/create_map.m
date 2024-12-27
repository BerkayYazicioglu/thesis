function map = create_map(map_size, edge_fcn, vertex_fcn)
    %% create a raw map from given map dimensions
    % o - o 
    % | X |  the graph is 8-connected
    % o - o
    % 
    % params:
    %   map_size = (1,2) node counts for X and Y axes
    %   edge_fcn(obj, cur, target) = function to allocate edge values
    %   vertex_fcn(obj, node_list) = returns a table allocating fields to the
    %                                node_list in given order
    
    if nargin < 2
        % no edge_fcn is given, default -1 to denote unknown distance
        edge_fcn = @(cur, target) -1;
    end
    if nargin < 3
        % no vertex_fcn is given, default set to current node value
        vertex_fcn = @(nodes) array2table(num2cell(nodes(:)), ...
                                                  'VariableNames', ...
                                                  {'Value'}, ...
                                                  'RowNames', ...
                                                  cellstr(num2str(nodes(:))));
    end

    % neighbor connectivity
    neighbors = [-1 -1; -1 0; -1 1;
                  0 -1;        0 1;
                  1 -1;  1 0;  1 1];

    numNodes = map_size(1) * map_size(2);
    adjacency = sparse(numNodes, numNodes);
 
    % iterate all nodes
    for node = 1:numNodes
        [node_row, node_col] = ind2sub(map_size, node);
        % find neighbors of the current node
        for k = 1:size(neighbors, 1)
            neighbor_row = node_row + neighbors(k, 1);
            neighbor_col = node_col + neighbors(k, 2);
            % Check if the new position is within boundaries
            if neighbor_row >= 1 ...
               && neighbor_row <= map_size(1) ...
               && neighbor_col >= 1 ...
               && neighbor_col <= map_size(2)
                 neighbor = sub2ind(map_size, neighbor_row, neighbor_col);
                 adjacency(node, neighbor) = edge_fcn(node, neighbor);
            end
        end
    end
    nodes = vertex_fcn(1:numNodes);
    map = graph(adjacency, nodes, 'upper');
end

