# nbvp_planning

The next best view planner is a real-time capable inspection path planner. From the current pose it expands a tree to find a next pose that gives a high information gain. This information gain reflects the exploration of space that is not yet (sufficiently) known. As the vehicle proceeds on the path, the tree is recomputed, taking in account the new information from the sensor.

# TODO

- IG account for occupied voxels as well
- Find way to map free cells, not only occupied
- Strategies to account for zero IG detection (e.g. go back along the path, increase size of sampling space, among others)

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

# Parameters

- system/v_max: Maximal translational speed
- system/dyaw_max: Maximal yaw-rate
- system/dv_max: Maximal translational acceleration (only enforced with euler integration tree extension)
- system/ddyaw_max: Maximal yaw acceleration (only enforced with euler integration tree extension)
- system/camera/pitch: Pitch of the camera sensor
- system/camera/horizontal: Horizontal opening of the field of view of the camera sensor
- system/camera/vertical: Vertical  opening of the field of view of the camera sensor

- nbvp/information_gain/free: Information gain for visible free volumes
- nbvp/information_gain/occupied: Information gain for visible occupied volumes
- nbvp/information_gain/unmapped: Information gain for visible unmapped volumes
- nbvp/information_gain/range: Maximum distance of volumes to be considered for the information gain computation
- nbvp/information_gain/degressive_coeff: Weighting factor for the summation of the node specific information gains along the branches of the tree
- nbvp/sampleHolonomic/extension_range: Maximum extension range for new tree branches when sampling for a holonomic system
- nbvp/RRT/initial_iterations: Initial number of iterations to build an RRT. If the best information gain is zero, the same number of iterations is performed again, in an extended sampling space
- nbvp/dt: Time step for integration and waypoint generation
- nbvp/RRT_extension: Wheather to use an RRT

- bbx/minX: Minimum x-coordinate value of the scenario. This only bounds the information gain computation and not the path planning
- bbx/minY: Minimum y-coordinate value of the scenario. This only bounds the information gain computation and not the path planning
- bbx/minZ: Minimum z-coordinate value of the scenario. This only bounds the information gain computation and not the path planning
- bbx/maxX: Maximum x-coordinate value of the scenario. This only bounds the information gain computation and not the path planning
- bbx/maxY: Maximum y-coordinate value of the scenario. This only bounds the information gain computation and not the path planning
- bbx/maxZ: Maximum z-coordinate value of the scenario. This only bounds the information gain computation and not the path planning
