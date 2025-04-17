function output = hybrid_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

cache = table({}, {}, [], {}, {}, {}, {}, [], ...
    'VariableNames', {'tasks', 'actions', 'u', 'u_map', 'u_search', 't', 'e', 'action_eval'});

if isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(0);
    output.pp_task_idx = [];
    output.action_eval = NaN;
    return
end

[pp, pp_task_idx] = milp_task_selector(robot, preprocessing);

if isempty(pp.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = seconds(0);
    output.pp_task_idx = [];
    output.action_eval = NaN;
    return;
end

% employ ga to solve the ordering problem
output = ga_task_allocator(robot, pp);
output.pp_task_idx = pp_task_idx;

end