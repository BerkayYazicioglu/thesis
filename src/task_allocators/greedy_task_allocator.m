function output = greedy_task_allocator(robot, preprocessing)
% preprocessing -> tasks, de, dt, outcomes (table with columns <nodes>, <values>, <actions>, <task_idx>)
%
% tasks -> ordered allocted tasks
% actions -> actions per tasks
% charge_flag -> return to the charger at the end of the tasks
% u -> utility of the selected allocation
% cache -> optimization cache

all_nodes = [preprocessing.tasks.node];
D = distance_matrix(robot, all_nodes, 1);
T = seconds(D./robot.speed) + preprocessing.dt;
E = robot.energy - D*robot.energy_per_m - preprocessing.de;

% get the maximum time
t_max = max(T);

cache = table({}, {}, [], {}, {}, {}, {}, ...
    'VariableNames', {'tasks', 'actions', 'u', 'u_map', 'u_search', 't', 'e'});

%% fitness function
function u = fitness(x)
    u = 0;
    % check for constraints
    if ~check_constraints(preprocessing.constraints{x}, T(x) + robot.time, E(x))
        return
    end

    rows = preprocessing.outcomes.task_idx == x;
    if any(rows)
        U = dictionary("map", 0, "search", 0); 
        cur_type = preprocessing.tasks(x).type;
        result = groupsummary(preprocessing.outcomes(rows,:), 'actions', 'sum', 'values');
        [U(cur_type), max_idx] = max(result.sum_values);
        action_ = result.actions(max_idx);
        % aggregate the utilites
        u = mcdm(robot.mission.mcdm, 1-T(x)/t_max, U("map"), U("search"));
        
        % update cache
        cache = [cache; {{x}, ...
                         {action_}, ...
                         u, ...
                         {U("map")}, ...
                         {U("search")}, ...
                         {T(x) + robot.time}}, ...
                         {E(x)}];
    end
    u = -u;
end
 
%% calculate fitness values
for i = 1:length(all_nodes)
    fitness(i);
end

% check if the robot needs to return to the charger
if isempty(cache)
    output.tasks = Task.empty;
    output.actions = string.empty;
    output.charge_flag = true;
    output.u = NaN;
    output.cache = cache;
    output.t_max = t_max;
else
    % find the best cache index
    cache.nodes = [cellfun(@(x) [preprocessing.tasks(x).node], cache.tasks, 'UniformOutput', false)];
    [max_u, max_row] = max(cache.u);
    output.tasks = preprocessing.tasks(cache.tasks{max_row});
    output.actions = cache.actions{max_row};
    output.charge_flag = false;
    output.u = max_u;
    output.cache = cache;
    output.t_max = t_max;
end

end

