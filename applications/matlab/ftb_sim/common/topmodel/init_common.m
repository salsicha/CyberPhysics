function init_common(varargin)
%INIT_COMMON loads data required for simulations by both plant and
%controller model

dict_name = 'common_data.sldd';
universal_constants = get_universal_constants;

add_entry_to_data_dict(dict_name, 'universal_constants', universal_constants, false);
end


