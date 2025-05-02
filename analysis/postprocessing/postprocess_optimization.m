function output = postprocess_optimization(missions)
    interval = 2 * 60;
    output = struct;
    data = struct;

    for ii = 1:length(missions)
        mission = missions(ii);
        data_types = ["t" "map" "search"];
        if ii == 1
            for i = 1:length(data_types)
                data.(data_types(i)).values = [];
                data.(data_types(i)).time = [];
            end
        end

        for iii = 1:length(mission.robots)
            robot = mission.robots(iii);
            % iterate through robot timesteps
            ts = robot.pp_outputs.keys;
            
            for i = 1:length(ts)
                pp = robot.pp_outputs(ts(i));
                if ~pp.charge_flag
                    ts_data = {[pp.cache.t_mcdm{:}]' ,...
                               [pp.cache.u_map{:}]' ,...
                               [pp.cache.u_search{:}]'};
                    % calculate which interval the data falls under
                    for j = 1:length(data_types)
                        data.(data_types(j)).values = [data.(data_types(j)).values; 
                                                       ts_data{j}];
                        data.(data_types(j)).time = [data.(data_types(j)).time; 
                                                     repmat(seconds(ts(i)), numel(ts_data{j}), 1)];
                    end
                end
            end
        end
    end

    
    for i = 1:length(data_types)
        time_intervals = 0:interval:max(data.(data_types(i)).time);
        output.(data_types(i)) = struct;
        output.(data_types(i)).time = zeros(length(time_intervals)-1, 1);
        output.(data_types(i)).mean = zeros(length(time_intervals)-1, 1);
        output.(data_types(i)).max = zeros(length(time_intervals)-1, 1);
        output.(data_types(i)).min = zeros(length(time_intervals)-1, 1);

        for ii = 1:length(time_intervals)-1
            output.(data_types(i)).time(ii) = mean(time_intervals(ii), time_intervals(ii+1));
            bin = data.(data_types(i)).values(...
                data.(data_types(i)).time >=  time_intervals(ii) & ...
                data.(data_types(i)).time <=  time_intervals(ii+1));
            output.(data_types(i)).mean(ii) = mean(bin);
            output.(data_types(i)).max(ii) = prctile(bin, 75);
            output.(data_types(i)).min(ii) = prctile(bin, 25);
        end
    end

end