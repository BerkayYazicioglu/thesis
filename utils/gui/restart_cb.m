%% restart callback
function restart_cb(app, event)
    new_mission = Mission(evalin('base', 'settings'));

    cla(evalin('base', 'gui.world'),'reset');
    cla(evalin('base', 'gui.population'),'reset');
    hold(evalin('base', 'gui.world'),'on');
    hold(evalin('base', 'gui.population'),'on');
    assignin('base', 'mission', new_mission);

    evalin('base', 'mission').plot(evalin('base', 'gui'));
    drawnow
end