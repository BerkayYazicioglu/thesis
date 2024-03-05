%% robot selection callback
function select_robot_cb(app, event)
   val = event.Value;
   gui = evalin('base', 'gui');
   if val == ""
       gui.x_init.Value = 0;
       gui.y_init.Value = 0;
   else
       p = evalin('base', 'settings').mission.robots.(val).p_init{1,1};
       gui.x_init.Value = p{1,1};
       gui.y_init.Value = p{1,2};
   end
end