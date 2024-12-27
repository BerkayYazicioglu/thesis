clc
clear
close

% Define the criteria group
% criteria has to be a one unique letter 
criteria = {'t', 'm', 's'};

% Generate all combinations
subsets = string.empty;
for k = 1:length(criteria)
    subset = nchoosek(criteria, k);
    for j = 1:size(subset, 1)
        subsets(end+1) = strjoin(sort(subset(j,:)), '');
    end
end

% MCDM table to be filled in by the user
save_file = 'MCDM_weights.mat';
MCDM.criteria = strjoin(sort(criteria), '');
MCDM.weights = table(subsets(:), zeros(length(subsets), 1), ...
    'VariableNames', {'key', 'weight'});
save(save_file, 'MCDM');