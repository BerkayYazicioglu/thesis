%% Discrete state estimation function for robots
% x(1) -> time
% x(2) -> current node
% x(3) -> energy
% x(4:N+3) -> considered tasks, idx of the robot that is assigned
%             -1: if it is a dummy task
%              0: if no assingment is done yet
%
% u(1) -> a connected node in the map

function z = robot_state_fcn(x, u, params)
    u(1) = round(u(1));
    % find the shortest path between x(2) <-> u
    [~, distance, ~] = params.map.shortestpath(num2str(x(2)), num2str(u(1)));
    
    % estimate the time and energy expenditure
    t = x(1) + distance / params.speed;
    e = x(3) - distance * params.energy_per_m;

    % sensors and tasks
    for i = 4:length(x)
        if x(i) == 0
           % performable task, get the capability from the current node
           task_node = params.considered_tasks(i-3);
           c = params.task_set.items(task_node).capability(params, u(1)); 
           % test if the task can be performed
           if c >= params.task_set.items(task_node).kill
               % the task can be performed, assign to the robot 
               % ============ WIP ==========
               x(i) = 1;
               % ===========================
           end
        end
    end

    % output
    z = zeros(size(x));
    z(1) = t;
    z(2) = u(1);
    z(3) = e;
    z(4:end) = x(4:end);
end