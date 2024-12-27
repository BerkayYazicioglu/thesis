classdef Task < handle
    %TASK Definition for the tasks
    
    properties
        % params fields
        pi_init string;
        pi_complete string;
        t_init duration;
        type string; 
        node string;

        % constructed fields
        cache table;
        R_k (1,:) string {mustBeVector}; % set of robots that can attempt
                                         % the task at timestep k
    end
    
    methods
        %% Constructor
        function obj = Task(params, t_init, node, robots)
            %TASK params (struct)->
            %   pi_init     : atomic proposition defining task initialization
            %   pi_complete : atomic proposition defining task completion
            %   type        : action associated with this task (map, search, patrol)
            % robots (Robot vector) -> set of all robots  
            % t_init (duration) -> task initialization time
            % node (string) -> task node

            % given config params
            obj.pi_init = params.pi_init;
            obj.pi_complete = params.pi_complete;
            obj.type = params.type;
            obj.t_init = t_init;
            obj.node = node;
            
            obj.cache = table('Size', [0 3], ...
                              'VariableTypes', {'string', 'string', 'string'}, ...
                              'VariableNames', {'robot_type', 'id', 'action'});
            % add robots that have the capability
            for r = 1:length(robots)
                c_types = [arrayfun(@(x) robots(r).(x).type, robots(r).capabilities)];
                if any(ismember(obj.type, c_types))
                    obj.R_k(end+1) = robots(r).id;
                end
            end
        end

        %% Predict the attempt of the task for a robot
        function [outcomes, de, dt] = predict(obj, robot)
            outcomes = table();
            de = 0;
            dt = seconds(0);
            if ~ismember(robot.id, obj.R_k)
                return;
            end
            % robot is allowed
            if obj.type == "map"
                outcomes = robot.mapper.predict(obj);
                de = robot.mapper.d_energy;
                dt = robot.mapper.t_s;
            elseif obj.type == "search"
                outcomes = robot.detector.predict(obj);
                de = robot.detector.d_energy;
                dt = robot.detector.t_s;
            end
            % Update R_k (remove robot if prediction is empty)
            if isempty(outcomes)
                obj.R_k(obj.R_k == robot.id) = [];
            end
        end
    end 
end

