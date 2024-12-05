% PLOT SAVED TRAJECTORY IN NORTH, EAST, UP (m)
% CAN BE USED TO PLOT ON TOP OF THIRD (LAST) PLOT IN mission.m
load xyzquat.mat
plot3(traj.Data(:,2)/.3048,traj.Data(:,1)/.3048,-traj.Data(:,3)/.3048)
hold on
