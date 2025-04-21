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
    data = fillmissing(data, 'previous');
    data_mean = mean(data{:, :}, 2);
    data_median = median(data{:, :}, 2);
    data_max = max(data{:, :}, [], 2);
    data_min = min(data{:, :}, [], 2);

    output = timetable(data.Time, data_mean, data_median, data_max, data_min, ...
        'VariableNames', {'mean', 'median', 'max', 'min'});
end