%%
clc
clear
close
 
addpath("single/"); 
addpath("../src/");
addpath("../src/utils/");
addpath("../src/gui/");

%% Settings
single_run_plots = ["distance"
                    "area"
                    "energy"
                    "victim"
                    "heatmap"
                    "optimization"
                    "task"
                    "utility"
                    "mission"];

%% Bind gui 
global single_gui result_path single_mission mission_files
result_path = "../simulation/";
single_gui = analysis_single_app;

single_gui.single_plot_select.Items = single_run_plots;
single_gui.single_plot_select.Value = single_run_plots(1);

files = {dir(result_path).name};
datasets = {};
for i = 3:length(files)
    datasets{end+1} = files{i};
end
single_gui.dataset_select.Items = datasets;
single_gui.dataset_select.Value = datasets{1};

mission_files = {dir(result_path + single_gui.dataset_select.Value).name};
mission_files = mission_files(3:end);
single_gui.mission_select.Items = mission_files;
single_gui.mission_select.Value = mission_files{1};

single_mission = load(result_path + single_gui.dataset_select.Value + "/" + single_gui.mission_select.Value + "/mission.mat").mission;
feval("single_" + single_gui.single_plot_select.Value + "_analysis", single_mission, single_gui);

single_gui.refresh.ButtonPushedFcn = @refresh;
single_gui.dataset_select.ValueChangedFcn = @dataset_select;
single_gui.mission_select.ValueChangedFcn = @mission_select;
single_gui.single_plot_select.ValueChangedFcn = @single_plot_select;
single_gui.single_plot_options.ValueChangedFcn = @single_options_select;

%% Callbacks
function refresh(app, event)
    global single_gui result_path single_mission
    files = {dir(result_path).name};
    datasets = {};
    for i = 3:length(files)
        datasets{end+1} = files{i};
    end
    single_gui.dataset_select.Items = datasets;

    feval("single_" + single_gui.single_plot_select.Value + "_analysis", single_mission, single_gui);  
end

%% 
function single_plot_select(app, event)
    global single_gui single_mission
    % run the currently selected plot with the new selections
    feval("single_" + single_gui.single_plot_select.Value + "_analysis", single_mission, single_gui);
end
%%
function dataset_select(app, event)
    global single_gui result_path single_mission
    mission_files = {dir(result_path + single_gui.dataset_select.Value).name};
    mission_files = mission_files(3:end);
    single_gui.mission_select.Items = mission_files;
    single_gui.mission_select.Value = mission_files{1};
    
    single_mission = load(result_path + single_gui.dataset_select.Value + "/" + single_gui.mission_select.Value + "/mission.mat").mission;
    feval("single_" + single_gui.single_plot_select.Value + "_analysis", single_mission, single_gui);
end

%% 
function mission_select(app, event)
    global single_gui result_path single_mission
    single_mission = load(result_path + single_gui.dataset_select.Value + "/" + single_gui.mission_select.Value + "/mission.mat").mission;
    feval("single_" + single_gui.single_plot_select.Value + "_analysis", single_mission, single_gui);
end

%% 
function single_options_select(app, event)
    global single_gui single_mission result_path
    single_mission = load(result_path + single_gui.dataset_select.Value + "/" + single_gui.mission_select.Value + "/mission.mat").mission;
    feval("single_" + single_gui.single_plot_select.Value + "_analysis", single_mission, single_gui);
end