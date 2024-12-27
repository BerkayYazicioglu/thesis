function U = mcdm(weights, t, m, s)
%MCDM Global utility calculation through MCDM Choquet integral
%   weights -> table with criteria combinations and their weights
%   t       -> normalized total time of the evaluated candidates
%   m       -> normalized total mapping utility of the evaluated candidates
%   s       -> normalized total search utility of the evaluated candidates

categories = ["t" "m" "s"];
U = zeros(1, length(t)); 

for k = 1:length(U)
    [sorted, idx] = sort([t(k) m(k) s(k)], 'descend');
    utilities = [sorted 0];
    u = 0;
    for i = 1:length(idx)
        % calculate the criterion ordering
        category = strjoin(sort(categories(idx(1:i))), '');
        weight = weights.weight(weights.key == category);
        u = u + weight*(utilities(i)-utilities(i+1));
    end
    U(k) = u;
end

end

