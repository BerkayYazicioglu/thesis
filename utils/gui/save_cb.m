%% save p_init entries callback
function save_cb(app, event)
    robot = evalin('base', 'gui.robot_select.Value');
    if robot == ""
        return
    end
    x = evalin('base', 'gui.x_init.Value');
    y = evalin('base', 'gui.y_init.Value');
    exp = sprintf("settings.mission.robots.%s.p_init{1,1}", robot);
    evalin('base', sprintf("%s = {%d, %d};", exp, x, y));
end