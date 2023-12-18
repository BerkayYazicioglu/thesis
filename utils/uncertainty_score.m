%% Uncertainty score generator
% generate uncertainty score functions wrt type of sensors and distance
%
% returns:
% epsilon(d) -> symfun with respect to distance and range

function epsilon = uncertainty_score(remote_sensing, max_range)
    syms d
    if remote_sensing
        % linear increase wrt distance 
        epsilon(d) = d ./ max_range;
    else
        % piecewise increase wrt distance from the middle
        cutoff = 1/2;
        epsilon(d) = piecewise(d < max_range * cutoff, ...
                               0, ...
                               d >= max_range * cutoff, ...
                               (d - max_range * cutoff)./(max_range * (1-cutoff)));
    end
end
