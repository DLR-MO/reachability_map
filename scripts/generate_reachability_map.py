import os
import sys
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import RobotModel, JointModelGroup
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import numpy as np
import argparse
import math
import rclpy
from pprint import pprint
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.node import Node

# TODO make arguments
# map size si given in x,y,z as tuples of lower and upper limit
MAP_SIZE = [[-2, 2], [-2, 2], [-2, 2]]  # m
VOXEL_SIZE = 0.01  # m
ANG_STEP_SIZE = math.radians(5)  # rad

voxels_in_x = math.ceil((MAP_SIZE[0][1] - MAP_SIZE[0][0]) / VOXEL_SIZE)
voxels_in_y = math.ceil((MAP_SIZE[1][1] - MAP_SIZE[1][0]) / VOXEL_SIZE)
voxels_in_z = math.ceil((MAP_SIZE[2][1] - MAP_SIZE[2][0]) / VOXEL_SIZE)


def rmap_entry_from_xyz(x, y, z):
    i_x = int((x - MAP_SIZE[0][0]) / VOXEL_SIZE)
    i_y = int((y - MAP_SIZE[1][0]) / VOXEL_SIZE)
    i_z = int((z - MAP_SIZE[2][0]) / VOXEL_SIZE)
    return i_x, i_y, i_z


def xyz_from_rmap_entry(i_x, i_y, i_z):
    x = float(i_x * VOXEL_SIZE + MAP_SIZE[0][0])
    y = float(i_y * VOXEL_SIZE + MAP_SIZE[1][0])
    z = float(i_z * VOXEL_SIZE + MAP_SIZE[2][0])
    return x, y, z


class ReachabilityMap:

    def __init__(self, robot_name, joint_group_name):
        rclpy.init()
        self.marker_node = Node("marker_publisher")
        self.marker_publisher = self.marker_node.create_publisher(
            MarkerArray, "/reachability_map", 10)
        # create a 3d numpy table which counts the number of configurations in this voxel
        self.rmap = np.zeros((voxels_in_x, voxels_in_y, voxels_in_z), dtype=int)
        # use the MoveIt python API to get a RobotState and the correct joint group
        moveit_config = (
            MoveItConfigsBuilder(
                "robot", package_name="elise_moveit_config")
            .moveit_cpp(
                file_path=get_package_share_directory("reachability_map")
                + "/moveit_planning_conf.yaml"
            )
            .to_moveit_configs()
        )
        self.robot = MoveItPy(node_name="moveit_py", config_dict=moveit_config.to_dict())
        self.robot_model = self.robot.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        self.joint_group_name = joint_group_name
        self.joint_group = self.robot_model.get_joint_model_group(self.joint_group_name)
        self.min_positions = []
        self.current_positions = []
        self.steps_per_joint = []
        # set all joints to their minimal position at the start
        i = 0
        for i in range(len(self.joint_group.active_joint_model_names)):
            print(self.joint_group.active_joint_model_names[i])
            print(f"min {self.joint_group.active_joint_model_bounds[i][0].min_position}")
            print(f"max {self.joint_group.active_joint_model_bounds[i][0].max_position}")
            min_position = self.joint_group.active_joint_model_bounds[i][0].min_position
            max_position = self.joint_group.active_joint_model_bounds[i][0].max_position
            # we ignore possible positions that are larger than one overall turn
            if min_position < math.tau:
                max_position = min(
                    self.joint_group.active_joint_model_bounds[i][0].max_position, math.tau/2)
            if max_position >= math.tau/2:
                min_position = max(
                    self.joint_group.active_joint_model_bounds[i][0].min_position, -math.tau/2)
            self.current_positions.append(
                self.joint_group.active_joint_model_bounds[i][0].min_position)
            self.steps_per_joint.append(round((max_position - min_position) / ANG_STEP_SIZE))
            self.min_positions.append(min_position)
            print(f"limited min {min_position}")
            print(f"limited max {max_position}")
            print("")
        pprint(self.current_positions)
        pprint(self.steps_per_joint)

        self.endeffector_name = self.joint_group.link_model_names[-1]
        # self.endeffector_name = "end_effector"
        self.pose_counter = 0
        self.poses_to_compute = 1
        for steps in self.steps_per_joint:
            self.poses_to_compute *= steps

        print(f"Will need to compute {self.poses_to_compute} configurations.")

    def generate_reachability_map(self):
        # loop through all joint configurations with defined step size
        def try_configurations_recursivly(joint_names, i):
            if len(joint_names) == 0:
                # update robot state to make sure the set joint positions are used
                self.robot_state.set_joint_group_positions(
                    self.joint_group_name, self.current_positions)
                self.robot_state.update()
                # verify if joints are in bounds
                # todo this is probably not working like this, number of joints does not match
                pose = self.robot_state.get_pose(self.endeffector_name)
                # pose = self.robot_state.get_frame_transform(self.endeffector_name)
                # print(pose)
                # i_x, i_y, i_z = rmap_entry_from_xyz(pose[0][3], pose[1][3], pose[2][3])
                i_x, i_y, i_z = rmap_entry_from_xyz(
                    pose.position.x, pose.position.y, pose.position.z)
                self.rmap[i_x, i_y, i_z] = self.rmap[i_x, i_y, i_z] + 1
                # print(str(self.current_positions))
                self.pose_counter += 1
                if self.pose_counter % 1000 == 0:
                    print(f"{100*self.pose_counter/self.poses_to_compute:.2f}%\r", end="")
                return
            for j in range(self.steps_per_joint[i]):
                self.current_positions[i] = self.min_positions[i] + j * ANG_STEP_SIZE
                try_configurations_recursivly(joint_names[1:], i+1)

        print("")
        try_configurations_recursivly(self.joint_group.active_joint_model_names, 0)
        print("finished")

    def send_marker_message(self):
        marker_array = MarkerArray()
        max_val = self.rmap.max()
        id = 0
        for i in range(self.rmap.shape[0]):
            for j in range(self.rmap.shape[1]):
                for k in range(self.rmap.shape[2]):
                    val = self.rmap[i, j, k]

                    if val > 0:
                        # Calculate color intensity based on value range
                        r = (max_val - val) / max_val
                        g = val / max_val

                        marker = Marker()
                        marker.header.frame_id = "base_link"  # todo take from robot model
                        marker.ns = "reachability_map"
                        marker.id = id
                        id += 1
                        marker.type = marker.SPHERE
                        MARKER_SCALE = 1.0  # todo make parameter
                        marker.scale.x = VOXEL_SIZE * MARKER_SCALE
                        marker.scale.y = VOXEL_SIZE * MARKER_SCALE
                        marker.scale.z = VOXEL_SIZE * MARKER_SCALE
                        x, y, z = xyz_from_rmap_entry(i, j, k)
                        marker.pose.position.x = x
                        marker.pose.position.y = y
                        marker.pose.position.z = z
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0
                        marker.color.r = r
                        marker.color.g = g
                        marker.color.b = 0.0
                        marker.color.a = 1.0  # todo make parameter
                        marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)
        print("published")
        while rclpy.ok():
            rclpy.spin(self.marker_node)


def main():
    parser = argparse.ArgumentParser(description="Generate Reachability Map")
    parser.add_argument("--robot-name", type=str, help="Name of the robot", default="elise")
    parser.add_argument("--joint-group-name", "-g", type=str,
                        help="Name of the joint group", default="all")
    args = parser.parse_known_args()[0]
    rmap = ReachabilityMap(args.robot_name, args.joint_group_name)
    rmap.generate_reachability_map()
    rmap.send_marker_message()

    print(rmap.rmap.max())

    print("really finished")


if __name__ == "__main__":
    main()
