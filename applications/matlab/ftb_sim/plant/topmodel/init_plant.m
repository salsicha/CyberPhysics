function init_plant(varargin)
%INIT_PLANT loads data required for plant simulations to plant data dictionary
%WARNING: Don't save the data dictionary while closing the project

dict_name = 'plant_data.sldd';
plant_sample_time_s = 1/250;
veh_params = get_vehicle_params;

add_entry_to_data_dict(dict_name, 'plant_sample_time_s', plant_sample_time_s, false);
add_entry_to_data_dict(dict_name, 'veh_params', veh_params, false);

end

