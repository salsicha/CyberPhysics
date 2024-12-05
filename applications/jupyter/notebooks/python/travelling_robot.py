import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from scipy import stats

"""
## Traveling Robot Problem

Visit a collection of points in the shortest path you can find. 
The catch? You have to "go home to recharge" every so often. 

### Details

* There are 5000 points distributed uniformly in [0, 1]
* The recharge station is located at (.5, .5)
* You cannot travel more than 3 units of distance before recharging
* You must start and end at the recharge station
"""

#############################
home = np.array([0.5, 0.5]) # home is the recharging station
max_charge = 3.0
#############################

# generate the points to visit uniformly in [0,1]
# recharging station is index 0
N = 5000
pts = np.vstack((home, np.random.rand(N,2)))

def check_order(pts, order):
	"""Check whether a given order of points is valid, and prints the total 
	length. You start and stop at the charging station.
	pts: np array of points to visit, prepended by the location of home
	order: array of pt indicies to visit, where 0 is home
	i.e. order = [0, 1, 0, 2, 0, 3, 0]"""

	print("Checking order")
	assert(pts.shape == (N + 1, 2)) # nothing weird
	assert(order[0] == 0) # start path at home
	assert(order[-1] == 0) # end path at home
	assert(set(order) == set(range(N + 1))) # all pts visited

	print("Assertions passed")

	# traverse path
	total_d = 0
	charge = max_charge
	last = pts[0,:]

	for i, idx in enumerate(order):
		pt = pts[idx, :]
		d = np.linalg.norm(pt - last)

		# update totals
		total_d += d
		charge -= d

		assert(charge > 0) # out of battery

		# did we recharge?
		if idx == 0:
			charge = max_charge

		# moving to next point
		last = pt

	# We made it to end! path was valid
	print("Valid path!")
	print(total_d)
	draw_path(pts, order)

def draw_path(pts, order):
	"""Draw the path to the screen"""
	path = pts[order, :]

	plt.plot(path[:,0], path[:,1])
	plt.show()


# Standard example:
# https://stackoverflow.com/questions/30552656/python-traveling-salesman-greedy-algorithm

# set up search grid
grid_size = 10
x = pts[:, 0]
y = pts[:, 1]
bins = np.linspace(0, 1.0, num = grid_size)

# create index pairs for each grid cell
index_pairs = []
for i in range(grid_size - 1):
	for j in range(grid_size - 1):
		index_pairs.append([i + 1, j + 1])

# get bin indices of points
binned_stats = stats.binned_statistic_2d(x, y, None, 'count', bins=[bins, bins], expand_binnumbers=True)
bn = binned_stats.binnumber

# vectorize distance calculation
home_dist = np.linalg.norm(pts - home, axis=1)
home_dist = np.concatenate((home_dist, np.array([0.0])))

# initialize odometry
order = []
odom_dist = 0.0

# create masks
masks = []
for pair in index_pairs:
	masks.append(np.where((bn[0] == pair[0]) & (bn[1] == pair[1]))[0])

# iterate over grid cells
for m, points_indices in enumerate(masks):
	indexed_points = pts[points_indices]
	last_point_index = points_indices[-1]

	# distance between random points in grid cell

	# Unsorted:
	# indexed_points = np.concatenate((home[np.newaxis, :], indexed_points))
	# inter_dist = np.linalg.norm(indexed_points[:-1:] - indexed_points[1::], axis=1)

	# Sorted distance to origin:
	# cell_sorted = np.argsort(home_dist[points_indices])
	# points_sorted = indexed_points[cell_sorted]
	# points_sorted = np.concatenate((home[np.newaxis, :], points_sorted))

	# Sorted nearest neighbor:
	from sklearn.neighbors import NearestNeighbors
	idx_points = np.concatenate((home[np.newaxis, :], indexed_points))
	nbrs = NearestNeighbors(n_neighbors=len(idx_points), algorithm='ball_tree').fit(idx_points)
	distances, indices = nbrs.kneighbors(idx_points)
	nn_order = [0]
	while len(nn_order) < indices.shape[0]:
		nns = indices[nn_order[-1]]
		for nn_idx in nns:
			if nn_idx not in nn_order:
				nn_order.append(nn_idx)
				break
	nn_order.pop(0)
	cell_sorted = np.array(nn_order) - 1
	points_sorted = idx_points[cell_sorted]
	points_sorted = np.concatenate((home[np.newaxis, :], points_sorted))

	# distance between points
	inter_dist = np.linalg.norm(points_sorted[:-1:] - points_sorted[1::], axis=1)

	# pad with home index to return home after completing a grid
	inter_dist = np.concatenate((inter_dist, np.array([home_dist[last_point_index]])))

	# Sorted:
	points_indices = points_indices[cell_sorted]
	points_indices = np.concatenate((np.array([0]), points_indices))
	points_indices = np.concatenate((points_indices, np.array([0])))

	# iterate over points in grid cell
	for i in range(inter_dist.shape[0]):
		idx = points_indices[i]
		order.append(idx)
		next_idx = points_indices[i + 1]

		# check that we can reach the next point
		next_dist = odom_dist + inter_dist[i] + home_dist[next_idx]
		if next_dist > max_charge or next_idx == 0:
			# if not, return home first
			odom_dist = home_dist[next_idx]
			order.append(0)
		else:
			# go to next point
			odom_dist += inter_dist[i]


check_order(pts, order)

