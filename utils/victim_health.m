%% Victim health function generator
% generate the health parameters of a victim using the settings in the
% config.yaml file
%
% returns:
% sigma -> symfun with respect to (t), health function
% sigma_init -> initial health %
% sigma_crit -> critical health % 

function [sigma, sigma_init, sigma_crit] = victim_health(params, destruction)
    % less initial and more critical health if destructed more
    sigma_crit = 0;
    sigma_init = 100;

    % longer it takes to reach crit if destructed less
    t_vic = seconds(duration(params.t_avg));

    % linear decay
    syms t
    sigma(t) = piecewise(t <= t_vic,  -(sigma_init-sigma_crit)/t_vic*t+sigma_init, ...
                         t > t_vic, sigma_crit);
end

