#include <functional>

#include <reachability_map_moveit/reachability_map_moveit.hpp>

using namespace std;


namespace reachability_map_moveit {


/**
 * @brief Constructor
 *
 * @param options node options
 */
ReachabilityMapMoveit::ReachabilityMapMoveit(const std::string& robot_name, const std::string& joint_group_name, const std::vector<std::vector<double>> map_size, const double voxel_size, const double ang_step_size) 
: Node("reachability_map_moveit"), 
robot_name_(robot_name), 
joint_group_name_(joint_group_name), 
map_size_(map_size), 
voxel_size_(voxel_size), 
ang_step_size_(ang_step_size), 
robot_model_loader_(shared_from_this()),
robot_model_(robot_model_loader_.getModel()),
grid_(voxel_size),
accessor_(grid_.createAccessor())
{

  //this->declareAndLoadParameter("param", param_, "TODO", true, false, false, 0.0, 10.0, 1.0);
  // callback for dynamic parameter configuration
  //parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&ReachabilityMapMoveit::parametersCallback, this, std::placeholders::_1));

  // prepare map and precompute some values
  //voxels_in_x_ = ceil((map_size_[0][1] - map_size_[0][0]) / voxel_size_);
  //voxels_in_y_ = ceil((map_size_[1][1] - map_size_[1][0]) / voxel_size_);
  //voxels_in_z_ = ceil((map_size_[2][1] - map_size_[2][0]) / voxel_size_);

  // create a Moveit Robot State to compute the forward kinematics
  robot_state_ = std::make_shared<moveit::core::RobotState>(moveit::core::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_group_ = robot_model_->getJointModelGroup(joint_group_name_);
  joint_names_ = joint_group_->getActiveJointModelNames();
  end_effector_name_ = joint_group_->getEndEffectorName();

  // get bounds without more than one turn per joint
  for (long unsigned int i = 0; i < size(joint_names_); ++i) {
    const moveit::core::JointModel* joint_model = joint_group_->getJointModel(joint_names_[i]);  
    const moveit::core::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
    const moveit::core::VariableBounds bounds = joint_bounds[0]; //todo this only works for single DOF joints
    double min_pos = bounds.min_position_;
    double max_pos = bounds.max_position_;
    if (min_pos < M_PI) {
        max_pos = std::min(max_pos, M_PI / 2);
    }
    if (max_pos >= M_PI / 2) {
        min_pos = std::max(min_pos, -M_PI / 2);
    }
    current_positions_.push_back(min_pos);
    steps_per_joint_.push_back(round((max_pos - min_pos) / ang_step_size_));
    min_positions_.push_back(min_pos);
  }

  // Compute the total number of configurations to evaluate
  poses_to_compute_ = 1;
  for (const auto& steps : steps_per_joint_) {
      poses_to_compute_ *= steps;
  }
  RCLCPP_INFO(this->get_logger(), "Will need to compute %lu configurations.", poses_to_compute_);

  // publisher for publishing outgoing messages
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/reachability_map", 10);
}

void ReachabilityMapMoveit::generate_reachability_map() {
  try_configurations_recursively(0);
  RCLCPP_INFO(this->get_logger(), "Finished generating the reachability map.");
}

void ReachabilityMapMoveit::try_configurations_recursively(int i) {
  if (i >= current_positions_.size()) {
      robot_state_->setJointGroupActivePositions(joint_group_, current_positions_);
      robot_state_->update();
      Eigen::Vector3d position = robot_state_->getGlobalLinkTransform(end_effector_name_).translation();
      //const int* i_x, i_y, i_z;
      //grid_entry_from_xyz(pose.position.x, pose.position.y, pose.position.z, i_x, i_y, i_z);
      //rmap[i_x][i_y][i_z]++;
      // this automatically initilized voxels with 0 if the voxel is not initialized
      auto* voxel = accessor_.value(grid_.posToCoord(position.x(), position.y(), position.z()), true);
      (*voxel)++;

      if (++pose_counter_ % 1000 == 0) {
          RCLCPP_INFO(this->get_logger(), "%.2f%% done", pose_counter_ * 100.0 / poses_to_compute_);
      }
      return;
  }

  for (int j = 0; j < steps_per_joint_[i]; ++j) {
      current_positions_[i] = min_positions_[i] + j * ang_step_size_;
      try_configurations_recursively(i + 1);
  }
}

void ReachabilityMapMoveit::send_marker_message() {
  visualization_msgs::msg::MarkerArray marker_array;
  uint32_t id = 0;
  uint32_t max_val = 0;
  auto maxVisitor = [this, &max_val](const uint32_t& value, const Bonxai::CoordT& coord) {
    max_val = std::max(max_val, value);
  };
  grid_.forEachCell(maxVisitor);

  //for (int i = 0; i < voxels_in_x_; ++i) {
  //    for (int j = 0; j < voxels_in_y_; ++j) {
  //        for (int k = 0; k < voxels_in_z_; ++k) {
  auto markerVisitor = [this, &marker_array, &max_val, &id](const uint32_t& value, const Bonxai::CoordT& coord) {
    //const uint32_t val = grid_[i][j][k];              
    if (value > 0) {
        double r = (max_val - value) / static_cast<double>(max_val);
        double g = value / static_cast<double>(max_val);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.ns = "reachability_map";
        marker.id = id++;
        marker.type = marker.SPHERE;
        marker.scale.x = voxel_size_;
        marker.scale.y = voxel_size_;
        marker.scale.z = voxel_size_;
        //double x, y, z;
        //xyz_from_grid_entry(i, j, k, x, y, z);
        marker.pose.position.x = coord.x;
        marker.pose.position.y = coord.y;
        marker.pose.position.z = coord.z;
        marker.pose.orientation.w = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }
  };
  grid_.forEachCell(markerVisitor);
  marker_pub_->publish(marker_array);
  RCLCPP_INFO(this->get_logger(), "Published reachability map markers.");
}

/**
 * @brief Declares and loads a ROS parameter
 *
 * @param name name
 * @param param parameter variable to load into
 * @param description description
 * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
 * @param is_required whether failure to load parameter will stop node
 * @param read_only set parameter to read-only
 * @param from_value parameter range minimum
 * @param to_value parameter range maximum
 * @param step_value parameter range step
 * @param additional_constraints additional constraints description
 */
/*
 template <typename T>
void ReachabilityMapMoveit::declareAndLoadParameter(const std::string& name,
                                                         T& param,
                                                         const std::string& description,
                                                         const bool add_to_auto_reconfigurable_params,
                                                         const bool is_required,
                                                         const bool read_only,
                                                         const std::optional<double>& from_value,
                                                         const std::optional<double>& to_value,
                                                         const std::optional<double>& step_value,
                                                         const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}
*/

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
/*
rcl_interfaces::msg::SetParametersResult ReachabilityMapMoveit::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}
*/

// Map helpers
void ReachabilityMapMoveit::grid_entry_from_xyz(double x, double y, double z, int& i_x, int& i_y, int& i_z) {
  i_x = static_cast<int>((x - map_size_[0][0]) / voxel_size_);
  i_y = static_cast<int>((y - map_size_[1][0]) / voxel_size_);
  i_z = static_cast<int>((z - map_size_[2][0]) / voxel_size_);
}

void ReachabilityMapMoveit::xyz_from_grid_entry(int i_x, int i_y, int i_z, double& x, double& y, double& z) {
  x = i_x * voxel_size_ + map_size_[0][0];
  y = i_y * voxel_size_ + map_size_[1][0];
  z = i_z * voxel_size_ + map_size_[2][0];
}
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  //TODO get this from arguments
  string robot_name = "elise";
  string joint_group_name_ = "all";
  vector<vector<double>> map_size = { {-2, 2}, {-2, 2}, {-2, 2} };
  double voxel_size = 0.01;
  double ang_step_size = M_PI / 36;   

  auto grid_node = make_shared<reachability_map_moveit::ReachabilityMapMoveit>(robot_name, joint_group_name_, map_size, voxel_size, ang_step_size);
  grid_node->generate_reachability_map();
  grid_node->send_marker_message();

  rclcpp::spin(grid_node);
  rclcpp::shutdown();
  return 0;
}
