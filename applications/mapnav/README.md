
# MapNav

This app cached height maps for a specified area. It then runs a ROS2 node in one of two environments:

1. Arduino App Lab brick 
2. BlueOS extension

The node receives position information, an unscaled depth image, and the latest kinematic information (velocity, acceleration, etc...), it matches the depth image to the geographic height map and produces a new position estimate.


