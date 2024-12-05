function params = get_vehicle_params(varargin)
%GET_VEHICLE_PARAMS Returns a struct with vehicle properties
%Ref 1: Minimum-complexity helicopter simulation math model
%
%NOTE: Some params are rough estimate from various sources and might need to be tuned once
%flight data is available

% MGW = 2268
params.mass_kg = 2000;
params.inertia_kgm2 = [7631.90 0 2264.22; 0 54232.72 0;2264.22 0 50436.43];

params.main_rotor = get_mr_params;


end

