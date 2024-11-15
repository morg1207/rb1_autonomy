#include "rb1_autonomy/plugins_bt/change_footprint.hpp"

ChangeFootprint::ChangeFootprint(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config) {

  node_ = rclcpp::Node::make_shared("bt_change_footprint");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_change_footprint]");
  init();
  initParameters();
}

void ChangeFootprint::init() {
  // Create publishers for local and global costmap footprints
  pubChangeFootprintLocal_ = node_->create_publisher<msgPolygon>("/local_costmap/footprint", 10);
  pubChangeFootprintGlobal_ = node_->create_publisher<msgPolygon>("/global_costmap/footprint", 10);
}

BT::NodeStatus ChangeFootprint::tick() {
  std::string type_change;

  // Check if the input parameter "port_int_type_change" is available
  if (!getInput<std::string>("port_int_type_change", type_change)) {
    throw BT::RuntimeError("missing required input [port_int_type_change");
  }

  // Case when type_change is "with_shelf"
  if (type_change == "with_shelf") {

    msgPolygon msgChangeFootprint;
    msgChangeFootprint.points.resize(4);
    geometry_msgs::msg::Point32 point_msg;
    
    // Define the points for the footprint of the robot with the shelf
    point_msg.x = shelf_length_ / 2;
    point_msg.y = shelf_width_ / 2;
    point_msg.z = 0.0;
    msgChangeFootprint.points[0] = point_msg;
    
    point_msg.x = shelf_length_ / 2;
    point_msg.y = -shelf_width_ / 2;
    point_msg.z = 0.0;
    msgChangeFootprint.points[1] = point_msg;
    
    point_msg.x = -shelf_length_ / 2;
    point_msg.y = -shelf_width_ / 2;
    point_msg.z = 0.0;
    msgChangeFootprint.points[2] = point_msg;
    
    point_msg.x = -shelf_length_ / 2;
    point_msg.y = shelf_width_ / 2;
    point_msg.z = 0.0;
    msgChangeFootprint.points[3] = point_msg;

    // Publish the footprint for both local and global costmaps
    pubChangeFootprintLocal_->publish(msgChangeFootprint);
    pubChangeFootprintGlobal_->publish(msgChangeFootprint);

    // Log the type of footprint being published
    RCLCPP_INFO(node_->get_logger(), "Publish footprint  [%s]", type_change.c_str());

    return BT::NodeStatus::SUCCESS;
  }

  // Case when type_change is "without_shelf"
  else if (type_change == "without_shelf") {

    msgPolygon msgChangeFootprint;
    msgChangeFootprint.points.resize(4);
    geometry_msgs::msg::Point32 point_msg;
    
    // Define the points for the footprint of the robot without the shelf
    point_msg.x = rb1_radius_;
    point_msg.y = rb1_radius_;
    point_msg.z = 0.0;
    msgChangeFootprint.points[0] = point_msg;
    
    point_msg.x = rb1_radius_;
    point_msg.y = -rb1_radius_;
    point_msg.z = 0.0;
    msgChangeFootprint.points[1] = point_msg;
    
    point_msg.x = -rb1_radius_;
    point_msg.y = -rb1_radius_;
    point_msg.z = 0.0;
    msgChangeFootprint.points[2] = point_msg;
    
    point_msg.x = -rb1_radius_;
    point_msg.y = rb1_radius_;
    point_msg.z = 0.0;
    msgChangeFootprint.points[3] = point_msg;

    // Publish the footprint for both local and global costmaps
    pubChangeFootprintLocal_->publish(msgChangeFootprint);
    pubChangeFootprintGlobal_->publish(msgChangeFootprint);

    // Log the type of footprint being published
    RCLCPP_INFO(node_->get_logger(), "Publish footprint  [%s]", type_change.c_str());

    return BT::NodeStatus::SUCCESS;
  }
  else {
    // Log an error message if the type_change is invalid
    RCLCPP_ERROR(node_->get_logger(), "The action type [%s] is not recognized. Please choose between -with_shelf- or -without_shelf-", type_change.c_str());

    return BT::NodeStatus::FAILURE;
  } 
}

void ChangeFootprint::initParameters() {
  // Parameters for the shelf length
  node_->declare_parameter("shelf_length", 1.0);
  shelf_length_ = node_->get_parameter("shelf_length").as_double();
  RCLCPP_INFO(node_->get_logger(), "Shelf length [%.3f]", shelf_length_);
  
  //----------------------------------------------------------

  // Parameters for the shelf width
  node_->declare_parameter("shelf_width", 0.8);
  shelf_width_ = node_->get_parameter("shelf_width").as_double();
  RCLCPP_INFO(node_->get_logger(), "Shelf width [%.3f]", shelf_width_);
  
  //----------------------------------------------------------

  // Parameters for the robot's radius
  node_->declare_parameter("rb1_radius", 0.25);
  rb1_radius_ = node_->get_parameter("rb1_radius").as_double();
  RCLCPP_INFO(node_->get_logger(), "Robot radius [%.3f]", rb1_radius_);
}
