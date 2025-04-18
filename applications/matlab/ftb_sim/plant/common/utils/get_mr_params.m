function params = get_mr_params(varargin)
%GET_MR_PARAMS Returns a struct with main rotor parameters needed for simulation
%Ref 1: Minimum-complexity helicopter simulation math model
%
%NOTE: Some params are rough estimate and might need to be tuned once
%flight data is available

params = struct;
% Positive for CCW
params.rotor_dir = 1;
% Main rotor radius
params.r_mr_m = 5.30;
% Main rotor disk area
params.rotor_disk_area_m2 = pi*params.r_mr_m^2;
% Main rotor precone
params.precone_mr_rad = deg2rad(2);
% Center of C.G X location w.r.t to FS from Manual
params.cg_x_m = 3.15;
% Flapping hinge offset - rough estimate from ref 1
params.hinge_offset_mr_m = 0.15;
% Pitch flap coupling, tangent of delta3 angle, assume a pitch command
% doesn't cause blade flap
params.K1 = 0;
% Initial Shaft tilt angle
params.shaft_tilt_rad = deg2rad(2);
% Blade twist
params.blade_twist_rad = 0;
% Blade chord length from manual
c_m = 0.273;
params.c_m = c_m;
% Mean profile drag coefficient- rough
% estimate from ref 1
params.cd0 =  0.010;
% Flapping inertia of a single blade about the flapping hinge - rough
% estimate from ref 1
Ib = 287.43;
% Typical value of lift curve slope
params.lift_curve_slope = 6;
params.gamma_mr = (params.lift_curve_slope*c_m*params.r_mr_m^4)/Ib;
% Number of blades
params.num_blades = 4;
% Blade solidity
params.sigma = params.num_blades*c_m/(pi*params.r_mr_m);
% Airspeed below which rotor wake effects are prominent for TPP dihedral
% effect
params.vtrans_mps = 15;

% Vertical distance of the hub from C.G in body axis, assume CG is constant
% for simplicity - Guesstimate from maintenance manual
params.h_hub_m = -1.3;

% Horizontal distance of the hub from C.G in body axis, assume CG is constant
% for simplicity - Guesstimate from maintenance manual
params.d_hub_m = 0.5;
end

