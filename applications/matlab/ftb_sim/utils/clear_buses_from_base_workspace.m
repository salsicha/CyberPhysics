function clear_buses_from_base_workspace
%CLEAR_BUSES_FROM_BASE_WORKSPACE Clears all buses from the base workspace

var_names_list = evalin('base', 'who');
for idx = 1:numel(var_names_list)
    bus_flag = evalin('base', ['class(' var_names_list{idx}, ');']);
    if strcmp(bus_flag, 'Simulink.Bus')
        evalin('base', ['clear ', var_names_list{idx}]);
    end
end
end

