How to Run:
1. Run mission.m in Matlab
2. Open stub_model.slx in Simulink
3. Run simulink model
4. Run plot_traj.m to see final 3D trajectory
5. Run convert2csv.m to generate CSV trajectory

File List:
convert2csv.m - convert xyzquat.mat to .csv
kml2lla.m - convert kml 3D route to 3 by x matrix, called by mission.m
mission.m - script to generate uam mission, needed by stub_model.slx
plot_traj.m - plot 3D trajectory from xyzquat.mat
SF-Candlestick.kml - sample scenario route, used by kml2lla.m from mission.m
stub_model.slx - stub model, needs mission.m
