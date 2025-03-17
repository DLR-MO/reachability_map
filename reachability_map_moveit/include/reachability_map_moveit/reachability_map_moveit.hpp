#pragma once

#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>


#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "bonxai/bonxai.hpp"

namespace reachability_map_moveit {

template <typename C> struct is_vector : std::false_type {};    
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};    
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;


class ReachabilityMapMoveit : public rclcpp::Node {

 public:

  ReachabilityMapMoveit(const std::string& robot_name, const std::string& joint_group_name, const std::vector<std::vector<double>> map_size, const double voxel_size, const double ang_step_size);
  void generate_reachability_map();
  void send_marker_message();

 private:
  void try_configurations_recursively(int i);
  void xyz_from_grid_entry(int i_x, int i_y, int i_z, double& x, double& y, double& z);
  void grid_entry_from_xyz(double x, double y, double z, int& i_x, int& i_y, int& i_z);


  /*template <typename T>
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
  */
  
  //double param_ = 1.0;
  //std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;
  //OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  const std::string robot_name_;
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
  std::string end_effector_name_;
  std::vector<double> current_positions_;
  std::vector<int> steps_per_joint_;
  std::vector<double> min_positions_;
  uint64_t poses_to_compute_ = 0;
  uint64_t pose_counter_ = 0;
  uint32_t voxels_in_x_;
  uint32_t voxels_in_y_;
  uint32_t voxels_in_z_;
};


}
