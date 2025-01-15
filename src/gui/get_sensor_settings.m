function  get_sensor_settings(gui, robot_id, sensor)
%GET_SENSOR_SETTINGS Get current settings of the sensors and display on gui

gui.sim.max_range.Value = gui.mission.settings.robots.(robot_id).capabilities.(sensor).max_range;
gui.sim.t_s.Value = gui.mission.settings.robots.(robot_id).capabilities.(sensor).t_s;
gui.sim.d_energy.Value = gui.mission.settings.robots.(robot_id).capabilities.(sensor).d_energy;
gui.sim.arc.Value = gui.mission.settings.robots.(robot_id).capabilities.(sensor).arc;
gui.sim.capability.Value = gui.mission.settings.robots.(robot_id).capabilities.(sensor).capability;

end

