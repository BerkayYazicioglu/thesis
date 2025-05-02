function output = postprocess_task(missions)
    output = struct;
    output.map = [];
    output.search = [];

    for iii = 1:length(missions)
        mission = missions(iii);
        history = mission.history.tasks;
        task_history = timetable(duration.empty(0,1), ...
        string.empty, string.empty, duration.empty, ...
        'VariableNames', {'task_type', 'task_node', 't_complete'});

        for i = 1:height(history)
            % check completed tasks
            if ~isempty(history.completed_nodes{i})
                nodes = history.completed_nodes{i};
                types = history.completed_types{i};
                for ii = 1:length(nodes)
                      idx = task_history.task_node == nodes(ii) & ...
                            task_history.task_type == types(ii);
                      idx = find(idx);
                      task_history.t_complete(idx) = history.Time(i);
                end
            end
            % check spawned tasks
            if ~isempty(history.spawned_nodes{i})
                nodes = history.spawned_nodes{i};
                types = history.spawned_types{i};
                for ii = 1:length(nodes)
                    task_history(end+1,:) = {types(ii), ...
                                             nodes(ii), ...
                                             seconds(nan)};
                    task_history.Time(end) = history.Time(i);
                end
            end
        end
        task_history.t_diff = task_history.t_complete - task_history.Time;
        output.map = [output.map; 
                      task_history.t_diff(task_history.task_type == "map")];
        output.search = [output.search; 
                      task_history.t_diff(task_history.task_type == "search")];
    end
end