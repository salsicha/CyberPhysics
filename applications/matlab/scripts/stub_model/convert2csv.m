load xyzquat.mat
csvwrite('xyzquat.csv',[traj.Time traj.Data])