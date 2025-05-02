function output = postprocess_area(missions)
    data = missions(1).robots(1).history(:, 'mapped_area');
    robots = missions(1).robots;
    for k = 2:length(robots)
        data = synchronize(data, robots(k).history(:,'mapped_area'), 'union');
    end

    for k = 2:length(missions)
        robots = missions(k).robots;
        for i = 1:length(robots)
            data = synchronize(data, robots(i).history(:, 'mapped_area'), 'union'); 
        end
    end
    [~, idx] = unique(data.Time);
    data = fillmissing(data(idx,:), 'previous');
    data_mean = mean(data{:, :}, 2);
    data_median = median(data{:, :}, 2);
    data_max = prctile(data{:,:}, 75, 2);
    data_min = prctile(data{:,:}, 25, 2);

    output = timetable(data.Time, data_mean, data_median, data_max, data_min, ...
        'VariableNames', {'mean', 'median', 'max', 'min'});
end