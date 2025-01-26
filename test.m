clc
clear
close

%% 

dataset_dir = 'results/simulation/';
dataset_names = ["ga_3" "ga_4" "ga_5"];
datasets = struct;
for i = 1:length(dataset_names)
    txt = readlines(dataset_dir + dataset_names(i) + '/ga.txt');
    disp(txt(1));

    gaGenerations = regexp(txt(2:end), 'ga generations:\s*(\d+)', 'tokens');
    funccounts = regexp(txt(2:end), 'funccount:\s*(\d+)', 'tokens');

    % Convert extracted tokens from cell array to numeric array
    datasets.(dataset_names(i)).generations = cellfun(@(x) str2double(x{1}), gaGenerations);
    datasets.(dataset_names(i)).funccounts = cellfun(@(x) str2double(x{1}), funccounts);
end

tiledlayout(2, 1);
nexttile
hold on
for i = 1:length(dataset_names)
    plot(datasets.(dataset_names(i)).generations)
end
legend(dataset_names);
hold off;

nexttile
hold on
for i = 1:length(dataset_names)
    plot(datasets.(dataset_names(i)).funccounts)
end
legend(dataset_names);
hold off;
