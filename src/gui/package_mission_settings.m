function settings = package_mission_settings(mission)
%PACKAGE_MISSION_SETTINGS Package all the current settings in the given
%               mission into the same structure as the config.yaml file

settings.t_end = string(mission.t_end);
settings.mcdm = mission.mcdm;
settings.PI_model = mission.settings.PI_model;
settings.prediction_errors = mission.prediction_errors;
% tasks
settings.tasks = mission.settings.tasks;
% charger
settings.charger = mission.settings.charger;
% robots
settings.robots = mission.settings.robots;

end

