# nbvp_planning

The next best view planner is a real-time capable inspection path planner. From the current pose it expands a tree to find a next pose that gives a high information gain. This information gain reflects the exploration of space that is not yet (sufficiently) known. As the vehicle proceeds on the path, the tree is recomputed, taking in account the new information from the sensor.

# Issues
*OcTree collision avoidance: While tracking the collision free path the rotorcraft flies into an occupied cell or while flying finds that the current position's cell is partially occupied. Consequently no obstacle free path to continue can be found.
*OcTree bounding box iterator does not work. It should be used to reduce the number of leaf cells to traverse in the IG computation. For some reason substitution of the normal iterator with bbx-iterator results in no leafs to iterate.
*Search of unmapped area tends to draw the system towards unknown interiors of structure, which can not be inspected. Also the unmapped outside area of the bounded scenario attracts the system. REMADY: Raycasting to unmapped voxel center. BUT: How to do this, as the OcTree does not allocate the unmapped nodes???

# Planner execution

To run the current version, compile the package nbvPlanner, as well as additionally required packages. To get these:

```sh
git clone https://github.com/ethz-asl/rotors_simulator.git
to be continued
```

Then run

```sh
roslaunch nbvPlanner firefly_exploration.launch
```

This produces a random walk, as the growing of the expansion trees as well as the information gain are based on randomness. Meaningful functions need to be implemented.

# Visualization in rviz

Add a display 'Marker' on the topic 'inspectionPath'. This displays the pose tree of the planning. To visualize the octomap add a display 'MarkerArray' on the topic '/occupied_cells_vis_array'
