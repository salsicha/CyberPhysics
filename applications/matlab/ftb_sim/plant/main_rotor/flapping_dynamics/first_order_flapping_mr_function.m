function [a1s_rad, b1s_rad] = first_order_flapping_mr_function(B1_mr_rad, A1_mr_rad, omega_mr_radps, ...
                       mr_thrust_N, as_mps, body_rates_radps, rho_kgpm3, params)
%FIRST_ORDER_FLAPPING_MR_FUNCTION implements first order flapping dynamics
%for the main rotor. A steady state assumption is used, which assumes
%that tip path plane (TPP) changes it's orientation instantaneously
%(transient flap motion is highly damped with time constant of approx
%16/gamma_mr). This implies transient response dies out after less than one
%revolution of the main rotor. Therefore a1dot = b1dot = 0. Where a1dot and
%b1dot is measured w.r.t to swash plate
%
%Inputs:
%B1_mr_rad          :       Cyclic pitch input
%A1_mr_rad          :       Cyclic roll input
%omega_mr_radps     :       Angular velocity of the main rotor
%mr_thrust_N        :       Rotor thrust from previous iteration
%as_mps             :       Airspeed (3D)
%body_rates_mps     :       Body angular rates
%rho_kgpm3          :       Air Density
%params             :       Main rotor and vehicle parameters struct
%
%Outputs:
%a1s_rad            :       Main rotor longitudinal tilt w.r.t to hub plane
%b1s_rad            :       Main rotor lateral tilt w.r.t to hub plane

% To avoid singularity
if(omega_mr_radps == 0)
    omega_mr_radps = eps;
    mr_thrust_N = 0;
end

% Rotor dir, 1 for CCW and -1 for CC
lambda = params.rotor_dir;

% Blade linkage cross-coupling
K1 = params.K1;

% Main rotor radius
r_mr_m = params.r_mr_m;

% Flap hing offset from the hub
e_mr_m = params.hinge_offset_mr_m;

% Main rotor solidity B*c/pi*R
sigma = params.sigma;

% Lift curve slope
a = params.lift_curve_slope;

% Rotor disk area
rotor_disk_area_m2 = params.rotor_disk_area_m2;

% Rotor thrust coefficient
CT = mr_thrust_N/(rho_kgpm3*(omega_mr_radps*r_mr_m)^2*rotor_disk_area_m2);

% Lateral MR dihedral derivative
lat_dihedral_deriv_mr = -2/(omega_mr_radps*r_mr_m)*(8*CT/(a*sigma) + sqrt(abs(CT/2)));
long_dihedral_deriv_mr = -lat_dihedral_deriv_mr;

% Lock number
gamma_mr = rho_kgpm3*params.gamma_mr;

% Change is natural frequency due to hinge offset
omega_f = gamma_mr*omega_mr_radps/16*(1 + 8*e_mr_m/(r_mr_m *3));

% Coupling due to hing offset
K2 = 0.75*(e_mr_m/r_mr_m)*(omega_mr_radps/omega_f);

% Total coupling
Kc = K1 + K2;

% Off axis flap rate coefficient
omega_off = omega_mr_radps/(1 + (omega_mr_radps/omega_f)^2);

% In axis flap rate coefficient
omega_in = omega_mr_radps*omega_off/omega_f;

u_mps = as_mps(1);
v_mps = as_mps(2);

if norm(as_mps) < params.vtrans_mps
    f_wake = 0;
else
    f_wake = 1;
end

T1 = (-omega_in - omega_off*Kc);
T2 = lambda*(-omega_in*Kc + omega_off);
T3 = (B1_mr_rad- long_dihedral_deriv_mr*u_mps*(1 + 2*f_wake));
T4 = (A1_mr_rad + lat_dihedral_deriv_mr*v_mps*(1 + f_wake));

p_radps = body_rates_radps(1);
q_radps = body_rates_radps(2);

% Longitudinal cyclic tilt angle w.r.t hub plane
a1s_rad = (T1* (q_radps + omega_in*T3 - lambda*omega_off*T4) + ...
      T2*(p_radps - omega_in*T4 - lambda*omega_off*T3))/(T1^2 + T2^2);
% Lateral cyclic tilt angle w.r.t hub plane
b1s_rad = (p_radps - omega_in*T4 - lambda*omega_off*T3 - a1s_rad*T2)/T1;


end

