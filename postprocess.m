clc
clear
close

addpath("analysis/"); 
addpath("analysis/single"); 
addpath("analysis/multi"); 
addpath("analysis/postprocessing"); 
addpath("src/gui/");
addpath("src");

%% post process settings
result_path = "results/"; 
save_path = "analysis/postprocessing/datasets/";
all_init_conds = [1   1; ...
                  50  1; ...
                  100 1; ...
                  100 50; ...
                  100 100; ...
                  50  100; ...
                  1   100; ...
                  1   50; ...
                  50  50];
sides = [2 4 6 8];
corners = [1 3 5 7];

%% load missions one by one
datasets = {dir(result_path).name};
datasets = datasets(3:end);

for i = 1:length(datasets)

files = {dir(result_path + datasets(i)).name};
files = files(3:end);
missions = Mission.empty();
init_conds = [];
% load all missions within the experiment
for ii = 1:length(files)
    missions(ii) = load(result_path + datasets{i} + "/" + files{ii} + "/mission.mat").mission;
    init_conds(ii, :) = cell2mat(missions(ii).q_init);
end
% calculate the post processing batches
batches = {};
names = string.empty;
for ii = 1:size(all_init_conds,1)
    batches{end+1} = find(ismember(init_conds, all_init_conds(ii,:), 'rows'));
    names(end+1) = sprintf('q_%d_%d', all_init_conds(ii,1), all_init_conds(ii,2));
end
batches{end+1} = find(ismember(init_conds, all_init_conds(sides,:), 'rows'));
names(end+1) = 'sides';
batches{end+1} = find(ismember(init_conds, all_init_conds(corners,:), 'rows'));
names(end+1) = 'corners';
batches{end+1} = [1:size(init_conds,1)]';
names(end+1) = 'all';

data = struct();

% postprocess each batch
for ii = 1:length(batches)
    M = [missions(batches{ii})];
    
    % area analysis
    data.(names(ii)).area = postprocess_area(M);
    
    % distance analysis
    data.(names(ii)).distance = postprocess_distance(M);

    % victim analysis
    data.(names(ii)).victim = postprocess_victim(M);

    % utility analysis
    data.(names(ii)).utility = postprocess_utility(M);
end

% save results
save(save_path + datasets{i} + '.mat', "data");

disp("postprocessing done for " + datasets{i});

clear data
clear M
clear missions
end

