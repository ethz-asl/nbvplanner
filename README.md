# nbvp_planning

The next best view planner is a real-time capable inspection path planner. From the current pose it expands a tree to find a next pose that gives a high information gain. This information gain reflects the exploration of space that is not yet (sufficiently) known. As the vehicle proceeds on the path, the tree is recomputed, taking in account the new information from the sensor.

# Planner execution

To run the current version, compile the package nbvPlanner and run

```sh
rosrun nbvPlanner nbvPlanner
```

This produces a random walk, as the growing of the expansion trees as well as the information gain are based on randomness. Meaningful functions need to be implemented.

# Visualization in rviz

Add a display for Pose on the topic inspectionPoint. This displays the current pose.
