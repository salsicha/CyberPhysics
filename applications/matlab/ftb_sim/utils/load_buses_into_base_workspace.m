function load_buses_into_base_workspace(dict_name)
%LOAD_BUSES_INTO_BASE_WORKSPACE Adds a provided entry to data dictionary
%
%Inputs:
%dict_name              : Name of the data dictionary with .sldd extension

dict_obj = Simulink.data.dictionary.open(dict_name);
h_des_data = getSection(dict_obj,'Design Data');

child_names_list = h_des_data.evalin('who');

for idx = 1:numel(child_names_list)
    h_entry = h_des_data.getEntry(child_names_list{idx});
    if isa(h_entry.getValue, 'Simulink.Bus')
        assignin('base', h_entry.Name, h_entry.getValue);
    end
end

close(dict_obj);
end
