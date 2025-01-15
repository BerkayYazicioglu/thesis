function get_robot_settings(gui, robot_id)
%GET_ROBOT_SETTINGS Get current settings of the robot and display on gui

gui.sim.crit_energy.Value = gui.mission.settings.robots.(robot_id).crit_energy;
gui.sim.energy_per_m.Value = gui.mission.settings.robots.(robot_id).energy_per_m;
gui.sim.charge_per_s.Value = gui.mission.settings.robots.(robot_id).charge_per_s;
gui.sim.max_step.Value = gui.mission.settings.robots.(robot_id).max_step;
gui.sim.speed.Value = gui.mission.settings.robots.(robot_id).speed;
gui.sim.height.Value = gui.mission.settings.robots.(robot_id).height;

sensors = fields(gui.mission.settings.robots.(robot_id).capabilities);
gui.sim.sensor_select.Items = [cellfun(@(x) string(x), sensors)];
gui.sim.sensor_select.Value = sensors{1};
get_sensor_settings(gui, robot_id, sensors{1});

gui.sim.preprocessing.Value = gui.mission.settings.robots.(robot_id).policy.preprocessing;
gui.sim.optimizer.Value = gui.mission.settings.robots.(robot_id).policy.optimizer;
gui.sim.num_tasks.Value = gui.mission.settings.robots.(robot_id).policy.num_tasks;
gui.sim.prediction_horizon.Value = gui.mission.settings.robots.(robot_id).policy.prediction_horizon;
gui.sim.control_horizon.Value = gui.mission.settings.robots.(robot_id).policy.control_horizon;

end

