# Receding Horizon Next Best View Planning

The next best view planner is a real-time capable exploration path planner. From the current pose it expands a tree to find a next pose that gives a high exploration gain. This gain reflects the exploration of space that is not yet (sufficiently) known. As the vehicle proceeds on the path, the tree is recomputed, taking into account the new information from the sensor.

This README gives a short overview. For more information refer to the [wiki](https://github.com/ethz-asl/nbvp_planning/wiki).

# Planner installation and execution

To run the current version, compile the package nbvplanner, as well as additionally required packages. To get these:

```sh
git clone https://github.com/ethz-asl/nbv_planning.git
git clone https://github.com/ethz-asl/catkin_simple.git
git clone https://github.com/ethz-asl/volumetric_mapping.git
git clone https://github.com/ethz-asl/gflags_catkin.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/minkindr.git
git clone https://github.com/ethz-asl/minkindr_ros.git
git clone https://github.com/ethz-asl/mav_comm.git
```

Moreover, make sure you have all the necessary octomap libraries:
```sh
apt-get install ros-<distro>-octomap-*
```

To be able to run the planner in the simulation environment, also clone the rotors repo:
```sh
git clone https://github.com/ethz-asl/rotors.git
```

For a simulation demo launch

```sh
roslaunch interface_nbvp_rotors flat_exploration.launch
```

Further instructions for the visualization of the exploration progress, as well as descriptions of the parameters can be found in the [wiki](https://github.com/ethz-asl/nbvp_planning/wiki).

# Credits

This algorithm was developed by [Andreas Bircher](mailto:bircher@gmx.ch) with the help and support of the members of the [Autonomous Systems Lab](http://www.asl.ethz.ch). The work was supported by the European Commission-funded project [AEROWORKS](http://www.aeroworks2020.eu/).

# Contact

You can contact us for any question or remark:
* [Andreas Bircher](mailto:bircher@gmx.ch)
* [Kostas Alexis](mailto:konstantinos.alexis@mavt.ethz.ch)
