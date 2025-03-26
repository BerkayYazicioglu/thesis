clearvars; clc;
addpath('../src/utils/')


settings = yaml.loadFile('../config.yaml');
MCDM = table(cellfun(@(x) string(sort(x)), fields(settings.mission.mcdm)), ...
             cellfun(@(x) settings.mission.mcdm.(x), fields(settings.mission.mcdm)), ...
             'VariableNames', {'key', 'weight'});

nvars = 100;
vals = linspace(0, 1, nvars);
u_map = zeros(nvars, nvars);
u_search = zeros(nvars, nvars);
x = zeros(nvars, nvars);
y = zeros(nvars, nvars);

for a = 1:nvars
    for t = 1:nvars
        u_map(a, t) = mcdm(MCDM, 1-vals(t), 1-vals(a), 0);
        u_search(a, t) = mcdm(MCDM, 1-vals(t), 0, 1-vals(a));
        x(a, t) = vals(t);
        y(a, t) = 1 - vals(a);
    end
end

global_min = min([u_map(:); u_search(:)]);
global_max = max([u_map(:); u_search(:)]);

tiledlayout(1, 2);
nexttile;
contourf(x, y, u_map);
xlabel('t');
ylabel('u');
clim([global_min global_max]);
title('MCDM u_{map}');

nexttile;
contourf(x, y, u_search);
xlabel('t');
ylabel('u');
title('MCDM u_{search}');
clim([global_min global_max]);
colorbar;
