function get_robot_settings(gui, robot_id)
%GET_ROBOT_SETTINGS Get current settings of the robot and display on gui

gui.sim.x_init.Value = gui.param_cache.robots.(robot_id).q_init{1};
gui.sim.y_init.Value = gui.param_cache.robots.(robot_id).q_init{2};
gui.sim.crit_energy.Value = gui.param_cache.robots.(robot_id).crit_energy;
gui.sim.energy_per_m.Value = gui.param_cache.robots.(robot_id).energy_per_m;
gui.sim.charge_s.Value = gui.param_cache.robots.(robot_id).charge_s;
gui.sim.max_step.Value = gui.param_cache.robots.(robot_id).max_step;
gui.sim.speed.Value = gui.param_cache.robots.(robot_id).speed;
gui.sim.height.Value = gui.param_cache.robots.(robot_id).height;

sensors = fields(gui.param_cache.robots.(robot_id).capabilities);
gui.sim.sensor_select.Items = [cellfun(@(x) string(x), sensors)];
gui.sim.sensor_select.Value = sensors{1};
get_sensor_settings(gui, robot_id, sensors{1});

gui.sim.preprocessing_select.Value = gui.param_cache.robots.(robot_id).policy.preprocessing;
gui.sim.optimizer_select.Value = gui.param_cache.robots.(robot_id).policy.optimizer;
gui.sim.num_tasks.Value = gui.param_cache.robots.(robot_id).policy.num_tasks;
gui.sim.prediction_horizon.Value = gui.param_cache.robots.(robot_id).policy.prediction_horizon;
gui.sim.control_horizon.Value = gui.param_cache.robots.(robot_id).policy.control_horizon;

end

