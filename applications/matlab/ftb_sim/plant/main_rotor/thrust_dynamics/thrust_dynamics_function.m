function [thrust_N, v_i_mps, v_hat_sq, idx] = thrust_dynamics_function(coll_mr_rad, omega_mr_radps, as_mps, ...
                            a1s_rad, b1s_rad, thrust_last_iter_N, v_hat_sq_last_iter, rho_kgpm3, params)
%THRUST_DYNAMICS_FUNCTION Computes main rotor thrust based on classical
%momentum theory
%
%Inputs:
%coll_mr_rad                    : Collective command
%omega_mr_radps                 : Main rotor angular velocity
%as_mps                         : Airspeed vector in body axis
%a1s_rad                        : Longitudinal flap angle
%b1s_rad                        : Lateral flap angle
%thrust_last_iter_N             : Computed thrust from last simulation
%                                 iteration
%v_hat_sq_last_iter             : Computed v_hat_sq from last simulation
%                                 iteration
%rho_kgpm3                      : Altitude corrected air density
%params                         : Main rotor parameters struct
%
%Outputs:
%thrust_N                       : Computed main rotor thrust
%v_i_mps                            : Induced velocity at main rotor
%v_hat_sq                       : Computed v_hat_sq
%idx                            : Iteration index when solution converged
%                                 for debug purpose

% Main rotor radius
r_mr_m = params.r_mr_m;

% Main rotor shaft tilt
shaft_tilt_rad = params.shaft_tilt_rad;

% Effective twist of the main rotor
blade_twist_rad = params.blade_twist_rad;

% Rotor disk area
rotor_disk_area_m2 = params.rotor_disk_area_m2;

% Lift curve slope
a = params.lift_curve_slope;

% Num blades
num_blades = params.num_blades;

% Mean blade chord length
c_m = params.c_m;

w_r = as_mps(3) + (a1s_rad + shaft_tilt_rad)*as_mps(1) - b1s_rad*as_mps(2);
w_blade= w_r + (2/3)*omega_mr_radps*r_mr_m*(coll_mr_rad + 0.75*blade_twist_rad);

% Initialize v_hat_sq and thrust at the simulation start
if thrust_last_iter_N == -1 && v_hat_sq_last_iter == -1
    v_hat_sq = sum(as_mps(1:2).^2) + as_mps(3)^2*w_r;
    thrust_N = 0.25*w_blade*rho_kgpm3*omega_mr_radps*r_mr_m^2*a*num_blades*c_m;
else
    v_hat_sq = v_hat_sq_last_iter;
    thrust_N = thrust_last_iter_N;
end

% Run 100 iteration for the thrust to converge based on inflow velocity,
% break out if convergence tolerance is met before 100 iterations
prev_thrust_N = 0;
thrust_conv_tol = 1e-4;
for idx = 1:100
    v_i_sq = sqrt((0.5*v_hat_sq)^2 + (0.5*thrust_N/rho_kgpm3/rotor_disk_area_m2)^2) - 0.5*v_hat_sq;
    v_i_mps = sqrt(abs(v_i_sq));
    v_hat_sq = sum(as_mps(1:2).^2) + as_mps(3)^2*(w_r - 2*v_i_mps);
    thrust_N = 0.25*(w_blade - v_i_mps)*rho_kgpm3*omega_mr_radps*r_mr_m^2*a*num_blades*c_m;
    if abs(prev_thrust_N - thrust_N) < thrust_conv_tol
        break;
    end
    prev_thrust_N = thrust_N;
end

end

