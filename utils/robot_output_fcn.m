%% Discrete output estimation function for robots
% x(1) -> time
% x(2) -> current node
% x(3) -> energy
% x(4:N+4) -> considered tasks
%
% u(1) -> a connected node in the map

function y = robot_output_fcn(x, u, params) 
    % elapsed time from the planning start
    dt = x(1) - seconds(params.t);
    
    score = 0;
    % sensors and tasks
    for i = 4:length(x)
        if x{i}.type == "none"
            continue
        end
        c = x{i}.capability(params, x{2});
        score = score + c;
    end

    y{1} = score / dt;
end
