%% Find the k-shortest paths from a node s to t in a graph g

function paths = find_k_shortest_paths(g, src, tgt, k, method)
    arguments
        g graph
        src char
        tgt cell
        k double {mustBePositive} = 1
        method string = "dijkstra"
    end

    paths = cell(length(tgt), k);

    ts = g.findnode([tgt; src]);
    targets = ts(1:end-1);
    s = ts(end);
    n = g.numnodes();

    for i = 1:length(targets)
        t = targets(i);

        if method == "dijkstra"
            q = CustomHeap(32000, 3);
            count = zeros(1, n);
            q.push([0 s 1]);
            lookup = dictionary(1, {s});   
            counter = 1;
            while count(t) < k
                lui = q.pop();
                l = lui(1);
                u = lui(2);
                idx = lui(3);
                if count(u) == k
                    lookup(idx) = [];
                    continue;
                end
                count(u) = count(u) + 1;
                walk = lookup(idx);
                walk = walk{1};
                if u == t
                    % found a path
                    paths{i, count(u)} = g.Nodes.Name(walk);
                end
                [edges, nodes] = g.outedges(u);
                for k = 1:length(edges)
                    counter = counter + 1;
                    e = edges(k);
                    q.push([l+g.Edges.Weight(e) nodes(k) counter]);
                    lookup(counter) = {[walk nodes(k)]};
                end
                lookup(idx) = [];
            end
        end
    end
end