function save_current_settings(gui)
%SAVE_CURRENT_SETTINGS Save all current settings on the gui to the cache

gui.param_cache.t_end = gui.sim.t_end.Value;
mcdm_row = gui.sim.mcdm_select.Value == gui.param_cache.mcdm.weights.key;
gui.param_cache.mcdm.weights.weight(mcdm_row) = gui.sim.mcdm_value.Value;

% charger
gui.param_cache.charger.policy.optimizer = gui.sim.charger_select.Value;
gui.param_cache.charger.policy.control_horizon = gui.sim.charger_control_horizon.Value;

% robot settings
robot_id = gui.sim.robot_select.Value;
gui.param_cache.robots.(robot_id).q_init = {gui.sim.x_init.Value, gui.sim.y_init.Value};
gui.param_cache.robots.(robot_id).crit_energy = gui.sim.crit_energy.Value;
gui.param_cache.robots.(robot_id).energy_per_m = gui.sim.energy_per_m.Value;
gui.param_cache.robots.(robot_id).charge_s = gui.sim.charge_s.Value;
gui.param_cache.robots.(robot_id).max_step = gui.sim.max_step.Value;
gui.param_cache.robots.(robot_id).speed = gui.sim.speed.Value;
gui.param_cache.robots.(robot_id).height = gui.sim.height.Value;

% sensor settings
sensor = gui.sim.sensor_select.Value;
gui.param_cache.robots.(robot_id).capabilities.(sensor).max_range = gui.sim.max_range.Value;
gui.param_cache.robots.(robot_id).capabilities.(sensor).t_s = gui.sim.t_s.Value;
gui.param_cache.robots.(robot_id).capabilities.(sensor).d_energy = gui.sim.d_energy.Value;
gui.param_cache.robots.(robot_id).capabilities.(sensor).arc = gui.sim.arc.Value;
gui.param_cache.robots.(robot_id).capabilities.(sensor).capability = gui.sim.capability.Value;

% policy settings
gui.param_cache.robots.(robot_id).policy.preprocessing = gui.sim.preprocessing_select.Value;
gui.param_cache.robots.(robot_id).policy.optimizer = gui.sim.optimizer_select.Value;
gui.param_cache.robots.(robot_id).policy.num_tasks = gui.sim.num_tasks.Value;
gui.param_cache.robots.(robot_id).policy.prediction_horizon = gui.sim.prediction_horizon.Value;
gui.param_cache.robots.(robot_id).policy.control_horizon = gui.sim.control_horizon.Value;

end

