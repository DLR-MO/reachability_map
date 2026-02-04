<!--
SPDX-FileCopyrightText: 2025 Marc Bestmann, German Aerospace Center (DLR)

SPDX-License-Identifier: MIT
-->

# MoveIt Reachability Maps

This package allows you to generate and visualize reachability maps for robots based on their their URDF and [MoveIt 2](https://github.com/moveit/moveit2) configuration.
It will try out all possible joint configurations (their sampling resolution can be chosen) and use forward kinematics to get the pose of the endeffector.
A [Bonxai](https://github.com/facontidavide/Bonxai) voxel map will be used to count all poses that end in each voxel. Thereby, it is possible to create a 3D voxel map that shows which areas the robot can reach and how many different configurations lead to this.

## Installation

Simply put this package in a colcon workspace and build it from source. It was developed for ROS 2 Jazzy, but should work for all latest ROS 2 versions.

## Usage

Make sure that you sourced the colcon workspaces that provide the MoveIt config of your robot as well as the workspace in which you build this package.
First, run the launch which starts the robots move_group node. If you used the MoveIt Setup Assistant you can run the `demo.launch.py`. The instructions below walk you through creating a reachability map for the [UR10e robot](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).

```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true launch_rviz:=false`
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e
```

Now you should see the standard RViz window with the MoveIt plugin.
You can now generate the reachability map by running

```shell
ros2 run reachability_map_moveit reachability_map_moveit PLANNING_GROUP VOXEL_SIZE SAMPLING_RESOLUTION
```

Here, the `PLANNING_GROUP` should be the name of the joint group that you want to use. The valid planning group names are defined in the SRDF, but you can also see them in the MoveIt RViz plugin under the planning tab.
The `VOXEL_SIZE` defines how large the voxel in the map will be.
The `SAMPLING_RESOLUTION` defines how far each joint is changed for sampling a new configuration and is defined in degree.
Beware that a small sampling resolution will lead to very long processing times.
You can add `--help` to see more options.


For the UR10e we can do a quick reachability map with the following command. The generated map is published as a marker array to `/reachability_map`for visualization in RViz, and is also exported as pointcloud for further processing.

```shell
ros2 run reachability_map_moveit reachability_map_moveit ur_manipulator 0.01 30 --export-pcd test.pcd`
```

> The topic is transient local, so you can show it in RViz without having to restart the script it (make sure to set the topic durability policy of the MarkerArray in RViz to `Transient Local`).

## Limitations

This software currently only works for planning groups with a single tip.
