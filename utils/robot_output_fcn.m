%% Discrete output estimation function for robots
% x(1) -> time
% x(2) -> current node
% x(3) -> energy
% x(4:N+3) -> considered tasks
%
% u(1) -> a connected node in the map

function y = robot_output_fcn(x, u, params) 
    % no utility of staying on the same node
    if x(2) == u(1)
        y(1) = 0;
        return
    end
    u(1) = round(u(1));
    % elapsed time from the planning start
    dt = x(1) - seconds(params.time);
    
    score = 0;
    % sensors and tasks
    for i = 4:length(x)
        if x(i) == 0
            % performable task, get the capability from the current node
           task_node = params.considered_tasks(i-3);
           c = params.task_set.items(task_node).capability(params, u(1)); 
           score = score + c;
        end
    end

    y(1) = score / dt;
end
