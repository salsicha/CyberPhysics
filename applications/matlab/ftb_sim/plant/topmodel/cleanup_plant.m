function cleanup_plant(varargin)
%CLEANUP_PLANT Cleans up plant data dictionary
Simulink.data.dictionary.closeAll('plant_data.sldd','-discard')
end

