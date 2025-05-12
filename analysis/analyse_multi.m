%%
clc
clear
close

addpath("multi/"); 
addpath("postprocessing/");
addpath("../src/");
addpath("../src/utils/");
addpath("../src/gui/");

%% Settings
multi_run_plots = ["area"
                   "distance"
                   "victim_count"
                   "victim_state"
                   "utility"
                   "optimization"
                   "task"];

%% Bind gui 
global multi_gui result_path all_init_conds
result_path = "../results/"; 
multi_gui = analysis_multi_app;
all_init_conds = [1   1; ...
                  25  1; ...
                  50  1; ...
                  70  1; ...
                  100 1; ...
                  100 25; ...
                  100 50; ...
                  100 75; ...
                  100 100; ...
                  75  100; ...
                  50  100; ...
                  25  100; ...
                  1   100; ...
                  1   75; ...
                  1   50; ...
                  1   25; ...
                  50  50];

multi_gui.multi_plot_select.Items = multi_run_plots;
multi_gui.multi_plot_select.Value = multi_run_plots(1);

files = {dir(result_path).name};
for i = 3:length(files)
    uitreenode(multi_gui.dataset_filter, 'Text', files{i});
end
expand(multi_gui.dataset_filter);

multi_gui.process.ButtonPushedFcn = @process;
multi_gui.refresh.ButtonPushedFcn = @refresh;

% gui.dataset_select.ValueChangedFcn = @dataset_select;
% gui.mission_select.ValueChangedFcn = @mission_select;
% gui.single_plot_options.ValueChangedFcn = @single_options_select;
% gui.single_plot_select.ValueChangedFcn = @single_plot_select;
% gui.type_switch.ValueChangedFcn = @switch_callback;
% 
% 
% gui.multi_plot_select.ValueChangedFcn = @multi_plot_select;
% gui.multi_group_select.ValueChangedFcn = @multi_group_select;


%% Callbacks
function refresh(app, event)
    global multi_gui all_init_conds
    groups = {'all'};
    for i = 1:length(all_init_conds)
        groups{end+1} = sprintf('q_%d_%d', all_init_conds(i,1), all_init_conds(i,2));
    end
    multi_gui.multi_group_select.Items = groups;
    multi_gui.multi_group_select.Value = groups(1);

    datasets = arrayfun(@(x) string(x.Text), multi_gui.dataset_filter.Parent.CheckedNodes);
    if datasets(1) == "Datasets"
        datasets(1) = [];
    end
    feval("multi_" + multi_gui.multi_plot_select.Value + "_analysis", datasets, multi_gui);
end

%%
function process(app, event)
    global multi_gui result_path all_init_conds

    save_path = "data/";
    datasets = arrayfun(@(x) string(x.Text), multi_gui.dataset_filter.Parent.CheckedNodes);
    if datasets(1) == "Datasets"
        datasets(1) = [];
    end
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
        batches{end+1} = [1:size(init_conds,1)]';
        names(end+1) = 'all';
        for ii = 1:size(all_init_conds,1)
            idx = find(ismember(init_conds, all_init_conds(ii,:), 'rows'));
            if isempty(idx)
                continue
            end
            batches{end+1} = idx;
            names(end+1) = sprintf('q_%d_%d', all_init_conds(ii,1), all_init_conds(ii,2));
        end
        
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

            % task analysis
            data.(names(ii)).task = postprocess_task(M);

            % optimization analysis
            data.(names(ii)).optimization = postprocess_optimization(M);
        end
        
        % save results
        save(save_path + datasets{i} + '.mat', "data");
        
        disp("postprocessing done for " + datasets{i});
        
        clear data
        clear M
        clear missions
    end
end