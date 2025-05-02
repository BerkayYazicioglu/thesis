function output = postprocess_victim(missions)
    output = struct;

    count_data = timetable(seconds(0), 0, 'VariableNames', {'count'});
    health_data = timetable(duration.empty(0,1), [], 'VariableNames', {'health'});
    for k = 1:length(missions)
        mission = missions(k);
        % go over victims and construct data
        victim_data = timetable();
        for i = 1:length(mission.world.victims)
            victim = mission.world.victims(i);
            if ~isempty(victim.history)
                history = victim.history;
                history.victim_id = i * ones(height(history), 1);
                victim_data = [victim_data; history];
            end
        end
        victim_data = sortrows(victim_data, 'Time');
        detected_data = victim_data(victim_data.status == 'detected', :);
        detected_data.count = [1:height(detected_data)]';
        
        count_data = synchronize(count_data, detected_data(:, "count"), 'union');
        health_data = [health_data; detected_data(:, "health")];
    end
    count_data(:, 1) = [];
    count_data(1, :) = array2timetable(zeros(1, size(count_data, 2)), 'RowTimes', seconds(0));
    [~, idx] = unique(count_data.Time);
    count_data = fillmissing(count_data(idx,:), 'previous');
    count_data.mean = mean(count_data{:, :}, 2);
    count_data.max = prctile(count_data{:,:}, 75, 2);
    count_data.min = prctile(count_data{:,:}, 25, 2);
    health_data = sortrows(health_data, 'Time');

    output.count = timetable(count_data.Time, count_data.mean, count_data.max, count_data.min, ...
        'VariableNames', {'mean', 'max', 'min'});
    output.health = health_data;
end