#pragma once

#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"

#include <argparse/argparse.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace reachability_map_moveit {

//template <typename C> struct is_vector : std::false_type {};    
//template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};    
//template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;


class ReachabilityMapMoveit {

 public:
  ReachabilityMapMoveit(const std::string& joint_group_name, const double voxel_size, const double ang_step_size);
  void generate_reachability_map();
  void send_marker_message(bool use_sphere, float scale);
  void spin();
  void save_reachability_map(std::string path);
  void load_reachability_map(std::string path);
  void export_pcd(std::string path);

 private:
  void try_configurations_recursively(long unsigned int i);
  uint32_t get_max_value();

  rclcpp::Node::SharedPtr node_;
  const std::string joint_group_name_;
  const std::vector<std::vector<double>> map_size_; //m  
  const double voxel_size_; //m
  const double ang_step_size_; //rad
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  robot_model_loader::RobotModelLoader robot_model_loader_;  
  const moveit::core::RobotModelPtr& robot_model_;
  Bonxai::VoxelGrid<uint32_t> grid_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_group_;
  Bonxai::VoxelGrid<uint32_t>::Accessor accessor_;
  std::vector<std::string> joint_names_;
  std::string tip_name_;
  std::vector<double> current_positions_;
  std::vector<int> steps_per_joint_;
  std::vector<double> min_positions_;
  uint64_t poses_to_compute_ = 0;
  uint64_t pose_counter_ = 0;
  uint32_t voxels_in_x_;
  uint32_t voxels_in_y_;
  uint32_t voxels_in_z_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;
};


}
