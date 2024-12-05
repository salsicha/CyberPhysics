function cleanup_common(varargin)
%CLEANUP_COMMON Cleans up common data dictionary
Simulink.data.dictionary.closeAll('common_data.sldd','-discard')
end

