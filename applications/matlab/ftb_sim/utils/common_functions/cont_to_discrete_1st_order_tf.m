function [num_array_d, den_array_d] = cont_to_discrete_1st_order_tf(num_array, den_array, sample_time_s)
%CONT_TO_DISCRETE_1ST_ORDER_TF For a given numerator and denominator of the
% first order continuous transfer function this function computes the numerator and
% denominator of the corresponding discrete transfer function using Tustin
% transformation
%
%Inputs:
%num_array              : Numerator of the TF in the form (B1*s + B0)/(A1*s + A0)
%den_array              : Denominator of the TF
%sample_time_s          : Sample time in seconds
%
%Outputs:
%num_array_d            : Numerator of the discrete TF in the form b0 + b1*z^-1/a0 + a1*z^-1
%den_array_d            : Denominator of the discrete TF

B1 = num_array(1);
B0 = num_array(2);

A1 = den_array(1);
A0 = den_array(2);

K = 2/sample_time_s;

temp = A0 + A1*K;

b0 = (B0 + B1*K)/temp;
b1 = (B0 - B1*K)/temp;

a0 = 1;
a1 = (A0 - A1*K)/temp;

num_array_d = [b0, b1];
den_array_d = [a0, a1];

end

