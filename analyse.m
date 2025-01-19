%%
clc
clear 
close

addpath("analysis/"); 
global gui result_path;
if exist('gui', 'Var')
    gui.delete;
end

%% Settings
result_path = "results/"; 
single_run_plots = ["distance"
                    "area"
                    "energy"
                    "victim"
                    "heatmap"
                    "optimization"
                    "task"
                    "utility"
                    "mission"];
multi_run_plots = ["distance"
                   "area"
                   "energy"
                   "victim"];

%% Bind gui 
gui = analysis_app;

gui.single_plot_select.Items = single_run_plots;
gui.single_plot_select.Value = single_run_plots(1);

gui.multi_plot_select.Items = multi_run_plots;
gui.multi_plot_select.Value = multi_run_plots(1);

files = {dir(result_path).name};
for i = 3:length(files)
    uitreenode(gui.dataset_filter, 'Text', files{i});
end
expand(gui.dataset_filter);

gui.save.ButtonPushedFcn = @save;
gui.dataset_select.ValueChangedFcn = @dataset_select;
gui.mission_select.ValueChangedFcn = @mission_select;
gui.single_plot_options.ValueChangedFcn = @single_options_select;
gui.single_plot_select.ValueChangedFcn = @single_plot_select;


%% Callbacks
function save(app, event)
    global gui result_path
    datasets = arrayfun(@(x) string(x.Text), gui.dataset_filter.Parent.CheckedNodes);
    if datasets(1) == "Datasets"
        datasets(1) = [];
    end
    % set dataset selectors
    gui.dataset_select.Items = datasets;
    gui.dataset_select.Value = datasets(1);

    % set mission selectors
    files = {dir(result_path + datasets(1)).name};
    gui.mission_select.Items = files(3:end);
    gui.mission_select.Value = files(3);

    % run the currently selected plot with the new selections
    if gui.type_switch.Value == "single"
        feval("single_" + gui.single_plot_select.Value + "_analysis", result_path, gui);
    else
    end
end

%% 
function single_plot_select(app, event)
    global gui result_path
    % run the currently selected plot with the new selections
    if gui.type_switch.Value == "single"
        feval("single_" + gui.single_plot_select.Value + "_analysis", result_path, gui);
    end
end

%%
function dataset_select(app, event)
    global gui result_path
    % set mission selectors
    files = {dir(result_path + gui.dataset_select.Value).name};
    gui.mission_select.Items = files(3:end);
    gui.mission_select.Value = files(3);

    % run the currently selected plot with the new selections
    if gui.type_switch.Value == "single"
        feval("single_" + gui.single_plot_select.Value + "_analysis", result_path, gui);
    end
end

%% 
function mission_select(app, event)
    global gui result_path
    % run the currently selected plot with the new selections
    if gui.type_switch.Value == "single"
        feval("single_" + gui.single_plot_select.Value + "_analysis", result_path, gui);
    end
end

%% 
function single_options_select(app, event)
    global gui result_path
    % run the currently selected plot with the new selection
    if gui.type_switch.Value == "single"
        feval("single_" + gui.single_plot_select.Value + "_analysis", result_path, gui, gui.single_plot_options.Value);
    end
end