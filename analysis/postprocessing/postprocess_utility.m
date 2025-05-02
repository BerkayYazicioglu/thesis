function output = postprocess_utility(missions)
    output = struct;

    for ii = 1:length(missions)
        mission = missions(ii);
        history = mission.history.pp;
        history.u_mcdm = nan(height(history), 1);
        history.u_action = nan(height(history), 1);
        history.action = strings(height(history), 1);
        for i = 1:height(history)
            robot = mission.robots([mission.robots.id] == history.robot(i));
            pp = robot.pp_outputs(history.Time(i));
            if ~pp.charge_flag
                idx = find(pp.u == pp.cache.u, 1, 'first');
                t_mcdm = pp.cache.t_mcdm{idx};
                u_map = pp.cache.u_map{idx};
                u_search = pp.cache.u_search{idx};
                % calculate partial mcdm output
                u = mcdm(robot.mission.mcdm, t_mcdm, u_map, u_search);
                history.u_mcdm(i) = sum(u(1: min(length(u), robot.policy.control_horizon)));
                history.u_action(i) = pp.action_eval;
                history.action(i) = extractBefore(pp.actions(1), "_");
            end
        end
        if ii == 1
            output.u_action = history(:, 'u_action');
            output.u_mcdm = history(:, 'u_mcdm');
        else
            output.u_action = synchronize(output.u_action, history(:, 'u_action'), 'union'); 
            output.u_mcdm = synchronize(output.u_mcdm, history(:, 'u_mcdm'), 'union'); 
        end
    end
    result = struct;

    [~, idx] = unique(output.u_action.Time);
    data = fillmissing(output.u_action(idx,:), 'previous');
    result.u_action.mean = mean(data{:, :}, 2);
    result.u_action.median = median(data{:, :}, 2);
    result.u_action.max = prctile(data{:,:}, 75, 2);
    result.u_action.min = prctile(data{:,:}, 25, 2);
    result.u_action.t = data.Time;

    [~, idx] = unique(output.u_mcdm.Time);
    data = fillmissing(output.u_mcdm(idx,:), 'previous');
    result.u_mcdm.mean = mean(data{:, :}, 2);
    result.u_mcdm.median = median(data{:, :}, 2);
    result.u_mcdm.max = prctile(data{:,:}, 75, 2);
    result.u_mcdm.min = prctile(data{:,:}, 25, 2);
    result.u_mcdm.t = data.Time;
    
    output = result;
end