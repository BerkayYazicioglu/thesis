%% Victim model

classdef Victim < handle
    %% Parameters
    properties
        id uint32; % location in victims set
        node uint32; % node on the graph
        location (1,2); % (m) [x y]
        sigma; % health function in time
        sigma_crit double {mustBeInRange(sigma_crit,0,100)}; % critical health percentage
        sigma_init double {mustBeInRange(sigma_init,0,100)}; % inital health percentage
        t_resc double {mustBePositive} = inf; % rescue time
        phi_det logical = false; % detection status

        color_init = [1 0 0]; % red
        color_crit = [0 0 0]; % black
        color_det = [0 1 0]; % green
        size = 25;
    end

    %% Methods
    methods
        function self = Victim(params)
            % params
            % id         -> position in the victims set
            % node       -> node in the graph
            % location   -> [x, y] in coordinates
            % sigma      -> health function % (t)
            % sigma_crit -> critical health %
            % sigma_init -> initial health %
           
            self.id = params.id;
            self.node = params.node;
            self.location = params.location;
            self.sigma_crit = params.sigma_crit;
            self.sigma_init = params.sigma_init;
            
            self.sigma = params.sigma;
        end

        %% Get current health
        function s = get_health(self, t)
            s = eval(self.sigma(seconds(t)));
        end

        %% Set detected with rescue time
        function detect(self, t_resc)
            self.phi_det = true;
            self.t_resc = seconds(t_resc);
        end
     
    end
end


