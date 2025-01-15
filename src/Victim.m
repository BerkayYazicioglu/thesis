classdef Victim < handle
    %VICTIM victim modeling
    
    properties
        % params fields
        idx double; 
        node string;
        t_end duration;
        
        % constructed fields
        t_detected duration;
        t_revisited duration;
        t_rescued duration;

        % history
        history timetable;
    end
    
    methods
        %% Constructor
        function obj = Victim(node, idx, t_end)
            %VICTIM 
            %   node : node id on the graph
            %   idx  : index on the victims list
            %   t_end: time duration before health reaches zero
            if nargin < 1
                return
            end
        
            obj.node = node;
            obj.idx = idx;
            obj.t_end = t_end;
            obj.t_detected = seconds(-inf);
            obj.t_revisited = seconds(-inf);
            obj.t_rescued = seconds(inf);

            obj.history = timetable(duration.empty(0,1),...
                string.empty, string.empty, duration.empty, [], string.empty, ...
                'VariableNames', {'robot_id', 'node', 'time', 'health', 'status'});
        end

        %% Set victim detected
        function detect(obj, robot)
            if robot.time < obj.t_rescued
                obj.t_revisited = robot.time;
                status = 'revisited';
                if obj.t_detected == seconds(-inf)
                    obj.t_detected = robot.time;
                    status = 'detected';
                end
                obj.history(robot.time, :) = {robot.id, ...
                                              robot.node, ...
                                              robot.time, ...
                                              obj.sigma(robot.time),...
                                              status};
            end
        end

        %% Set victim rescued
        function rescue(obj, robot)
            if robot.time < obj.t_rescued
                obj.t_rescued = robot.time;
                status = 'rescued';
                obj.history(robot.time, :) = {robot.id, ...
                                              robot.node, ...
                                              robot.time, ...
                                              obj.sigma(robot.time),...
                                              status};
            end 
    end

        %% Health status
        function h = sigma(obj, t)
            % Linearly decreasing function of time
            h = max(-100./seconds(obj.t_end).*seconds(t) + 100, 0);
        end
    end
end

