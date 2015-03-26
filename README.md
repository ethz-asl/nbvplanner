# nbvp_planning

The next best view planner is a real-time capable inspection path planner. From the current pose it expands a tree to find a next pose that gives a high information gain. This information gain reflects the exploration of space that is not yet (sufficiently) known. As the vehicle proceeds on the path, the tree is recomputed, taking in account the new information from the sensor.

# TODO

- Check for bounding box of octomap. Areas outside remain unmapped
- Extend tree nodes for the distance covered by the system in one step
- OcTree collision avoidance: Multiple rays to secure approximated tube and not just a line
- IG account for occupied voxels as well
- Compute global information gain (ratio of explored area to total area)
- Structured tree expansion (e.g. RRT)
- Strategies to account for zero IG detection (e.g. go back along the path, increase size of sampling space, among others)
- Complete list of required packages

# Planner execution

To run the current version, compile the package nbvplanner, as well as additionally required packages. To get these:

```sh
git clone https://github.com/ethz-asl/rotors_simulator.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/glog_catkin.git
apt-get install ros-<distro>-octomap-*
```
Please leave a note if you find that something is missing.

Note: some of the required packages are under development and therefore constant change. Seamless interaction with the latest version can not be guaranteed but the interfaces are updated every now and then. (Last update Wednesday, 18.3.2015)

Then run

```sh
roslaunch nbvplanner firefly_exploration.launch
```

# Visualization in rviz

- Add a display 'Marker' on the topic 'inspectionPath'. This displays the pose tree of the planning.
- To visualize the octomap used for planning, add a display 'MarkerArray' on the topic '/occupied_cells_vis_array'.
- For a high resolution octomap of the mapped areas add a display 'MarkerArray' on the topic '/analysis/occupied_cells_vis_array'.
