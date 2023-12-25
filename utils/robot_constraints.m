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


function ceq = robot_constraints(X, U, data, params)    
    p = data.PredictionHorizon;
    ceq = zeros(p, 1);
    for i = 1:p
        candidates = [];
        % get the candidate nodes from the current available tasks
        for j = 4:data.NumOfStates
            % performable task
            if X(i, j-3) == 0
               task_node = params.considered_tasks(j-3);
               % get the sensor type of the task and add the FoV to
               % candidates
               pi = params.task_set.items(task_node).pi;
               fov = params.(pi).get_all(task_node, params.world);
               candidates = [candidates;
                             fov(:)];

            end
        end
        candidates = unique(candidates);
        % exclude already visited nodes from previous steps
        candidates = setdiff(candidates, X(1:i, 2));
        % check if the corresponding input is one of the candidates
        ceq(i) = any(candidates == U(i)) - 1;
    end
end
