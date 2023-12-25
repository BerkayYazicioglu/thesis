%% Discrete constraints for robot trajectory optimization
% x(1) -> time
% x(2) -> current node
% x(3) -> energy
% x(4:N+3) -> considered tasks
%
% u(1) -> a connected node in the map
%
% X(p+1, N+3) -> state trajectory along the prediction horizon p
%                the first row is the current state values
% U(p+1) -> input trajectory along the prediction horizon
%           U(end,:) = U(end-1,:) 
% data -> additional signal struct 
% params -> robot object
%
% ceq(N_const) -> equality contraints as a column vector


function J = robot_cost_fcn(X, U, e, data, params)    
    p = data.PredictionHorizon;
    utility = 0;
    for i = 1:p+1
        % the utility is the sum of the outputs of each prediction step
        y = robot_output_fcn(X(i,:)', U(i,:)', params)';
        utility = utility + y(1);
    end
    % maximize utility, minimize J
    J = -utility;
end
