#include "rb1_autonomy/plugins_bt/publish_transform.hpp"



PublishTransform::PublishTransform(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){

  node_ = rclcpp::Node::make_shared("bt_publish_transform");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_publish_transform]");
  init();
  initParameters();
}

BT::NodeStatus PublishTransform::tick(){

  // Transform utils
  tfUtils_ = std::make_shared<TransformUtils>(node_);

  msgPoint position_object;
  bool get_target_nav;

  if (!getInput<msgPoint>("port_int_position_object",position_object)) {

    throw BT::RuntimeError("missing required input [port_int_position_object]");
  }
  if (!getInput<bool>("port_int_get_target_nav",get_target_nav)) {

    throw BT::RuntimeError("missing required input [port_int_get_target_nav]");
  }
  // **********************************************
  std::string to_frame = "robot_odom";
  std::string from_frame = "robot_front_laser_base_link";

  msgTransform t;
  t = tfUtils_->getTransform(tfBuffer_,to_frame,from_frame);
  t = tfUtils_->findTransform(t, position_object);

  frame_parent_ = "robot_odom";
  frame_child_ = "shelf_frame";
  tfUtils_->sendTransform(tfBroadcaster_,t,frame_child_,frame_parent_); 

  // **********************************************
  t = changeTransform(-(distance_aproach_target_shelf_ + radius_rb1_), 0);

  frame_parent_ = "shelf_frame";
  frame_child_ = "target_front_frame";
  tfUtils_->sendTransform(tfBroadcaster_,t, frame_child_,frame_parent_);

  // **********************************************
  t = changeTransform(( large_shelf_ / 2), 0);

  frame_parent_ = "shelf_frame";
  frame_child_ = "shelf_center_frame";
  tfUtils_->sendTransform(tfBroadcaster_,t, frame_child_,frame_parent_);

  // **********************************************
  if(get_target_nav){
    t = changeTransform(-(distance_aproach_target_shelf_ + radius_rb1_) - 0.2, 0);

    frame_parent_ = "shelf_frame";
    frame_child_ = "target_nav_frame";
    tfUtils_->sendTransform(tfBroadcaster_,t, frame_child_,frame_parent_);

    to_frame = "map";
    from_frame = "target_nav_frame";
    t = tfUtils_->getTransform(tfBuffer_,to_frame,from_frame);
    sendTargetNavGoal(t);
    
    msgPose target_nav;
    target_nav = sendTargetNavGoal(t);
    setOutput<msgPose>("port_out_target_nav_goal", target_nav);
  }

  return BT::NodeStatus::SUCCESS;
}

void PublishTransform::init(){

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  tfBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

}

msgTransform PublishTransform::changeTransform(float x_des, float y_des){
  geometry_msgs::msg::TransformStamped t;

  t.transform.translation.x = x_des;
  t.transform.translation.y = y_des;
  t.transform.translation.z = 0;

  t.transform.rotation.x = 0;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.transform.rotation.w = 1;
  return t;
}

msgPose PublishTransform::sendTargetNavGoal(msgTransform& transform){

  msgPose target_nav;
  target_nav.position.x = transform.transform.translation.x;
  target_nav.position.y = transform.transform.translation.y;
  target_nav.position.z = transform.transform.translation.z;
  target_nav.orientation = transform.transform.rotation;

  return target_nav;
}

void PublishTransform::initParameters(){


  // Declare and get the distance to approach the target shelf
  node_->declare_parameter("distance_aproach_target_shelf", 0.10);
  distance_aproach_target_shelf_ = node_->get_parameter("distance_aproach_target_shelf").as_double();
  RCLCPP_INFO(node_->get_logger(), "Distance to approach target shelf [%.3f]", distance_aproach_target_shelf_);
  //----------------------------------------------------------    

  // Declare and get the radius of the RB1 robot
  node_->declare_parameter("radius_rb1", 0.25);
  radius_rb1_ = node_->get_parameter("radius_rb1").as_double();
  RCLCPP_INFO(node_->get_logger(), "RB1 radius [%.3f]", radius_rb1_);
  //----------------------------------------------------------

  // Declare and get the shelf length
  node_->declare_parameter("shelf_length", 0.65);
  large_shelf_ = node_->get_parameter("shelf_length").as_double();
  RCLCPP_INFO(node_->get_logger(), "Shelf length [%.3f]", large_shelf_);
  //----------------------------------------------------------

}
