# reachability_map_moveit

This package allows you to generate reachability maps for robots based on their their URDF and MoveIt configuration.
It will try out all possible joint configurations (their sampling resolution can be choosen) and use forward kinematics to get the pose of the endefector.
A bonxai voxel map will be used to count all poses that end in each voxel. Thereby, it is possible to create a 3D voxel map that shows which areas the robot can reach and how many different configurations lead to this.

## Installation

Simply put this package in a colcon workspace and build it.

## Usage

Make sure that you sourced the colcon workspaces that provide the MoveIt config of your robot as well as the workspace in which you build this package.
First, run the launch which starts the robots move_group node. If you used the MoveIt Setup Assistant you can run the `demo.launch.py`. As example we will use the (UR10e robot)[https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver].
`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true launch_rviz:=false`
`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e`

Now you should see the standard RViz window with the MoveIt plugin.
You can now run the generation of the reachability map with
`ros2 run reachability_map_moveit reachability_map_moveit PLANNING_GROUP VOXEL_SIZE SAMPLING_RESOLUTION`
The planning group is needs to be the name of the moveit joint group that you want to use. You can find the possible names in the SRDF, but you can also see them in the MoveIt RViz plugin in the planning tab.
The voxel size defines how large the voxel in the map will be.
The sampling resolution defines how far each joint is changed for sampling a new configuration and is defined in degree. Beware that a small sampling resolution will lead to very long processing times.
Use `--help` to see more options.
For the UR10e we can do a quick reachability map with the following command. It will be both send as a marker topic for RViz but also exported as pointcloud (which can be used for further processing).
`ros2 run reachability_map_moveit reachability_map_moveit ur_manipulator 0.01 30 --export-pcd test.pcd`

The reachability map is send as a marker array message to RViz on the topic `/reachability_map`. The topic is transient local, so you can show it in RViz without having to restart the script it (make sure to set the topic durability policy of the MarkerArray in RViz to `Transient Local`).

## Limitatins

This software currently only works for planning groups with a single tip.