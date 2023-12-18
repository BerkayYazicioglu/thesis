%% Discrete state estimation function for robots
% x(1) -> time
% x(2) -> current node
% x(3) -> energy
% x(4:N+4) -> considered tasks
%
% u(1) -> a connected node in the map

function z = robot_state_fcn(x, u, params)
    % find the shortest path between x(2) <-> u
    [path, distance, edges] = params.map.shortestpath(num2str(x{2}), num2str(u{1}));
    
    % estimate the time and energy expenditure
    t = x{1} + distance / params.speed;
    e = x{3} - distance * params.energy_per_m;

    % sensors and tasks
    for i = 4:length(x)
        if x{i}.type == "none"
            continue
        else
           c = x{i}.capability(params, u{1}); 
           if c >= 0.99
               x{i}.robot = params.id;
               % WIP
               x{i}.type = "none";
           end
        end
    end

    % output
    z = cell(size(x));
    z{1} = t;
    z{2} = u{1};
    z{3} = e;
    z{4:end} = x{4:end};
end