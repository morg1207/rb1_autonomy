#include "rb1_autonomy/plugins_bt/nav_poses.hpp"


NavPoses::NavPoses(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){


  node_ = rclcpp::Node::make_shared("bt_nav_poses");
  node_ = rclcpp::Node::make_shared("bt_nav_poses");
  init();
  initParameters();

}

void NavPoses::init(){


}

BT::NodeStatus NavPoses::tick(){

  std::string number_pose;
  if (!getInput<std::string>("port_int_nav_poses",number_pose)) {

    throw BT::RuntimeError("missing required input [port_int_nav_poses]");
  }

  RCLCPP_INFO(node_->get_logger(), "String recieved, %s ", number_pose.c_str());
  if(number_pose.find("set pose",0)== 0){
    number_pose.erase(0,9);
  }
  else{
    RCLCPP_ERROR(node_->get_logger(), "Enter the position in the following format, set pose x ");
    return BT::NodeStatus::FAILURE;
  }
  std::cout << number_pose << std::endl;
  msgPoseNav msg_nav_goal;

  int number_pos = atoi(number_pose.c_str())-1;

  msg_nav_goal.position.x = nav_poses_x_[number_pos];
  msg_nav_goal.position.y = nav_poses_y_[number_pos];
  msg_nav_goal.position.z = 0.0;
  msg_nav_goal.orientation.x = 0.0;
  msg_nav_goal.orientation.y = 0.0;
  msg_nav_goal.orientation.z = 0.0;
  msg_nav_goal.orientation.w = 1.0;

  RCLCPP_INFO(node_->get_logger(), "Sending goal x = [%.3f] y= [%.3f]",nav_poses_x_[number_pos],nav_poses_y_[number_pos] );
  setOutput<msgPoseNav>("port_out_target_nav_goal", msg_nav_goal);


  return BT::NodeStatus::SUCCESS;
}

void NavPoses::initParameters(){

  std::stringstream ss;
  ss << "[";

  node_->declare_parameter<std::vector<double>>("nav_poses_x", {0.0, 1.0, 2.0});
  nav_poses_x_ = node_->get_parameter("nav_poses_x").as_double_array();

  for(auto item= nav_poses_x_.begin(); item != nav_poses_x_.end(); item++){
    ss << *item;  
    if( item != nav_poses_x_.end()-1) ss<< ", ";
  }
  ss << "]";
  RCLCPP_INFO_STREAM(node_->get_logger(), "nav poses x: " << ss.str());
  //----------------------------------------------------------    

  ss.str("");
  ss << "[";
  node_->declare_parameter<std::vector<double>>("nav_poses_y", {0.0, 1.0, 2.0});
  nav_poses_y_ = node_->get_parameter("nav_poses_y").as_double_array();

  for(auto item= nav_poses_y_.begin(); item != nav_poses_y_.end(); item++){
    ss << *item;  
    if( item != nav_poses_y_.end()-1) ss<< ", ";
  }
  ss << "]";
  RCLCPP_INFO_STREAM(node_->get_logger(), "nav poses y: " << ss.str());
  //----------------------------------------------------------    

  ss.str("");
  ss << "[";
  node_->declare_parameter<std::vector<double>>("nav_poses_z", {0.0, 1.0, 2.0});
  nav_poses_z_ = node_->get_parameter("nav_poses_z").as_double_array();
  
  for(auto item= nav_poses_z_.begin(); item != nav_poses_z_.end(); item++){
    ss << *item;  
    if( item != nav_poses_z_.end()-1) ss<< ", ";
  }
  ss << "]";
  RCLCPP_INFO_STREAM(node_->get_logger(), "nav poses z: " << ss.str());

}



