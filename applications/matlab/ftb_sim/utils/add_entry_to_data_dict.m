function add_entry_to_data_dict(dict_name, entry_name, entry_value, save_flag)
% ADD_ENTRY_TO_DATA_DICT Adds a provided entry to data dictionary
%
%Inputs:
%dict_name              : Name of the data dictionary
%extension
%
%entry_name             : Entry name
%entry_value            : Entry value
%save_flag              : Whether to save the dictionary or not. Usually
%dictionary shouldn't be saved and populated before running the simulation
%with desired parameters
dict_obj = Simulink.data.dictionary.open(dict_name);
des_data = getSection(dict_obj,'Design Data');

addEntry(des_data, entry_name, entry_value);

if save_flag
    saveChanges(dict_obj);
end

close(dict_obj);
end

