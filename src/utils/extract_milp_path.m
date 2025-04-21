function path = extract_milp_path(result, variables)

X = reshape(result.x(variables.X(:)), [variables.n variables.n]);
idx = 1;
path = 1;
while true
    next = find(X(idx, :) == 1);
    if isempty(next)
        break
    end
    path(end+1) = next;
    idx = next;
end

end

