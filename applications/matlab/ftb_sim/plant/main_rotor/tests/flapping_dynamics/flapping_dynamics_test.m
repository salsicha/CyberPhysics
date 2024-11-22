function flapping_dynamics_test(varargin)
%FLAPPING_DYNAMICS_TEST Runs a simple tests for Main Rotor Flapping
%Dynamics library.

% Load and run simulation
load_system('flapping_dynamics_test_harness');
simOut = sim('flapping_dynamics_test_harness','SimulationMode','normal',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','yout',...
            'SaveFormat', 'Dataset');
outputs = simOut.yout;
close_system('flapping_dynamics_test_harnes', 0)

% Get all the outputs
time = outputs{1}.Values.Time;
% Flap angles
a1_s = outputs{1}.Values.Data;
b1_s = outputs{2}.Values.Data;

% Cyclic inputs
B1_mr_rad = outputs{3}.Values.Data;
A1_mr_rad = outputs{4}.Values.Data;

% Rotor angular velocity
omega_mr_radps = outputs{5}.Values.Data;

% air speeds
u_mps = outputs{7}.Values.Data(:, 1);
w_mps = outputs{7}.Values.Data(:, 3);

% body rates
p_radps = outputs{8}.Values.Data(:, 1);
q_radps = outputs{8}.Values.Data(:, 2);

zero_B1_idx = find(~B1_mr_rad);
zero_A1_idx = find(~A1_mr_rad);

zero_omega_idx = find(~omega_mr_radps);

zero_u_idx = find(~u_mps);
zero_w_idx = find(~w_mps);

zero_p_idx = find(~p_radps);
zero_q_idx = find(~q_radps);

% When all inputs and excitations are zero then flap angles should be zero
zero_all_idx = intersect(zero_B1_idx, intersect(zero_A1_idx, ...
            intersect(zero_omega_idx, intersect(zero_u_idx, ...
            intersect(zero_w_idx, intersect(zero_p_idx, zero_q_idx))))));

assert(all(~a1_s(zero_all_idx)), 'a1_s value should be zero')
assert(all(~b1_s(zero_all_idx)), 'b1_s value should be zero')

% Max flap angles check
non_zero_B1_idx = find(B1_mr_rad);
non_zero_A1_idx = find(A1_mr_rad);

[~, a1_max_idx] = max(abs(a1_s));
[~, b1_max_idx] = max(abs(b1_s));

time_a1_max_idx = time(a1_max_idx);
time_b1_max_idx = time(b1_max_idx);

% Assert that maximum flap angles happen when cyclic inputs are provided
assert( (min(time(non_zero_B1_idx)) <= time_a1_max_idx) && ...
    (time_a1_max_idx <= max(time(non_zero_B1_idx))), ...
    'Time of max a1_s occurance is incorrect')

assert( (min(time(non_zero_A1_idx)) <= time_b1_max_idx) && ...
    (time_b1_max_idx <= max(time(non_zero_A1_idx))), ...
    'Time of max b1_s occurance is incorrect')

disp('Tests Passed ..........')
end


