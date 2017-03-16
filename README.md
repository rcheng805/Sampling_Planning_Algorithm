# Sampling_Planning_Algorithm

Implements Sampling-Based Motion Planning Algorithm, which plans a path for a circular robot from an initial point to a goal point while avoiding polygonal obstacles. 

Code is run from Path_Planner_c1.m where initial point, robot radius, obstacles, and goal point can be defined.

checkCollion.m - Checks for collision between robot and obstacles
sample_points.m - Samples the next point to potentially be added to our motion planning graph
check_min_distance.m - Finds node in existing graph, which is closest to the randomly sampled point
node_placement_collision.m - Places the new node such that it is not too far from the existing tree, and it does not intersect any obstacles
