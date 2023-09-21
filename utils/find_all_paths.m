function output = find_all_paths(graph, start, depth, adj_fcn)
    %% Find all paths from a start node with given depth
    % graph: undirected graph to search in
    % start: start node ID
    % depth: how many nodes to include in each path
    % adj_fcn(graph, node): function to find the neighbors of a node


    path = stack({start});
    visited = {start};

    step_happened = false;
    final_break = false;
    current_depth = 1;
    output = {};

    while true
        % Until path is filled with depth nodes
        while path.size() < depth
            current_node = path.peek();
            step_happened = false;
            current_depth = path.size();
        
            adj_nodes = adj_fcn(graph, current_node);
            % If no nodes were visited at current_depth
            % The next level of visited must include nodes from the previous level
            % that are in path or adjacent to the current_node
            if length(visited) < current_depth 
                visited_prev = visited{current_depth-1};
                in_path_or_adj = unique([path.content(true) adj_nodes]);
                visited_new = intersect(visited_prev, in_path_or_adj);
                visited{current_depth} = visited_new;
            end
        
            for i = 1:length(adj_nodes)
                adj_node = adj_nodes(i);
                if ~ismember(adj_node, visited{current_depth})
                    path.push(adj_node);
                    visited{current_depth} = [visited{current_depth} adj_node];
                    step_happened = true;
                    current_depth = current_depth + 1;
                    break;
                end
            end
        
            if ~step_happened
                % No new possible step from current_node
                % All adjacent nodes to current_node are visited
                final_break = current_node == start;
                break;
            end
        end

        if final_break break; end

        % Step happened and length(path) == depth
        if step_happened
            output{end+1} = path.content(true);
            % Path was written to output
            % We delete the last element from path
            % to search for other paths on the next iteration
            path.pop();
        else
            % If no steps available for given depth,
            % We don't need visited at this dept
            visited(current_depth) = [];

            dead_end = path.pop();
            current_depth = current_depth - 1;
            current_node = path.peek();

            % If all possible paths were explored from this node,
            % it is considered as visited
            visited{current_depth} = [visited{current_depth}, dead_end];

            % Nodes in visited at the current level have to
            % be adjacent to the last node in the path or be in the path
            visited_prev = visited{current_depth};
            in_path_or_adj = unique([path.content(true), adj_fcn(graph, current_node)]);
            visited{current_depth} = intersect(visited_prev, in_path_or_adj);
        end
    end
end
