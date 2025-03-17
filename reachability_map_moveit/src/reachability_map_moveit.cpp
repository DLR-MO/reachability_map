#include <functional>

#include <reachability_map_moveit/reachability_map_moveit.hpp>

using namespace std;


namespace reachability_map_moveit {


/**
 * @brief Constructor
 *
 * @param options node options
 */
ReachabilityMapMoveit::ReachabilityMapMoveit(const std::string& robot_name, const std::string& joint_group_name, const double voxel_size, const double ang_step_size) 
: node_(rclcpp::Node::make_shared("reachability_map_moveit")), 
robot_name_(robot_name), 
joint_group_name_(joint_group_name), 
voxel_size_(voxel_size), 
ang_step_size_(ang_step_size), 
robot_model_loader_(node_),
robot_model_(robot_model_loader_.getModel()),
grid_(voxel_size),
accessor_(grid_.createAccessor())
{
  // create a Moveit Robot State to compute the forward kinematics
  robot_state_ = std::make_shared<moveit::core::RobotState>(moveit::core::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_group_ = robot_model_->getJointModelGroup(joint_group_name_);
  joint_names_ = joint_group_->getActiveJointModelNames();
  end_effector_name_ = joint_group_->getEndEffectorName();
  end_effector_name_ = "end_effector"; //todo hack

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
    
    RCLCPP_INFO(node_->get_logger(), "Joint %s min pos %f, max pos %f configurations.\n", joint_names_[0].c_str(), min_pos, max_pos);
  }

  // Compute the total number of configurations to evaluate
  poses_to_compute_ = 1;
  for (const auto& steps : steps_per_joint_) {
      poses_to_compute_ *= steps;
  }
  RCLCPP_INFO(node_->get_logger(), "Will need to compute %lu configurations.", poses_to_compute_);

  // publisher for publishing outgoing messages
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/reachability_map", 10);
}

void ReachabilityMapMoveit::spin(){
    rclcpp::spin(node_); //todo this whole method is hacky
}

void ReachabilityMapMoveit::generate_reachability_map() {
  try_configurations_recursively(0);
  RCLCPP_INFO(node_->get_logger(), "\nFinished generating the reachability map.");
}

void ReachabilityMapMoveit::try_configurations_recursively(long unsigned int i) {
  if (i >= current_positions_.size()) {
      robot_state_->setJointGroupActivePositions(joint_group_, current_positions_);
      robot_state_->update();
      Eigen::Vector3d position = robot_state_->getGlobalLinkTransform(end_effector_name_).translation();
      // this automatically initilized voxels with 0 if the voxel is not initialized
      auto* voxel = accessor_.value(grid_.posToCoord(position.x(), position.y(), position.z()), true);
      (*voxel)++;

      if (++pose_counter_ % 1000 == 0) {
          std::cout << "\r" << pose_counter_ * 100.0 / poses_to_compute_ << "% done" << std::flush;
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

  auto markerVisitor = [this, &marker_array, &max_val, &id](const uint32_t& value, const Bonxai::CoordT& coord) {    
        double r = (max_val - value) / static_cast<double>(max_val);
        double g = value / static_cast<double>(max_val);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.ns = "reachability_map";
        marker.id = id++;
        marker.type = marker.CUBE;
        marker.scale.x = voxel_size_ * 0.8;
        marker.scale.y = voxel_size_ * 0.8;
        marker.scale.z = voxel_size_ * 0.8;
        auto pos = grid_.coordToPos(coord);
        marker.pose.position.x = pos.x;
        marker.pose.position.y = pos.y;
        marker.pose.position.z = pos.z;
        marker.pose.orientation.w = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);    
  };
  grid_.forEachCell(markerVisitor);
  marker_pub_->publish(marker_array);
  RCLCPP_INFO(node_->get_logger(), "Published reachability map markers.");
}
}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  //TODO get this from arguments
  string robot_name = "elise";
  string joint_group_name_ = "endo";
  double voxel_size = 0.005;
  double ang_step_size = M_PI / 360;

  auto grid_node = make_shared<reachability_map_moveit::ReachabilityMapMoveit>(robot_name, joint_group_name_, voxel_size, ang_step_size);
  grid_node->generate_reachability_map();
  grid_node->send_marker_message();

  grid_node->spin();
  rclcpp::shutdown();
  return 0;
}
