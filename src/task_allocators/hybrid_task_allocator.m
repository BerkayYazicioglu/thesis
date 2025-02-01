function output = hybrid_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

if isempty(preprocessing.tasks)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = table({}, {}, [], {}, {}, {}, {}, 'VariableNames', {'tasks', 'actions', 'u', 'u_map', 'u_search', 't', 'e'});
    output.t_max = seconds(0);
    return
end

pp = milp_task_selector_new(robot, preprocessing);
% employ ga to solve the ordering problem
output = ga_task_allocator(robot, pp);
end