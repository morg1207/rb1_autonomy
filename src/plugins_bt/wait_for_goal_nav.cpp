#include "rb1_autonomy/plugins_bt/wait_for_goal_nav.hpp"


WaitForGoalNav::WaitForGoalNav(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){


  node_ = rclcpp::Node::make_shared("bt_wait_for_goal_nav");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_wait_for_goal_nav]");
  flag_pose_recieved_ = false;
  init();

}

void WaitForGoalNav::init(){

  subPoseNav_ = node_->create_subscription<msgPose>("nav_goal_for_discharge",1, std::bind(&WaitForGoalNav::callbackPoseNav, this, _1));

}

BT::NodeStatus WaitForGoalNav::tick(){

  int nav_goal_timeout_sec;

  if (!getInput<int>("port_int_nav_goal_timeout_sec", nav_goal_timeout_sec)) {

    throw BT::RuntimeError("missing required input [port_int_nav_goal_timeout_sec");
  }
  // wait for nav goal
  auto start_time = node_->now();
  auto elapse_time = rclcpp::Duration(nav_goal_timeout_sec,0);

  auto rate = rclcpp::Rate(20);
  while(rclcpp::ok() && elapse_time > (node_->now()-start_time)){

    if(flag_pose_recieved_){
        RCLCPP_INFO(node_->get_logger(),"Pose has been received");

        setOutput<msgPose>("por_out_goal_nav", nav_goal_);
        return BT::NodeStatus::SUCCESS; 
    }
    RCLCPP_INFO(node_->get_logger(),"Waiting for nav goal");
    rclcpp::spin_some(node_);
    rate.sleep();
  }
  RCLCPP_ERROR(node_->get_logger(), "Pose has not been received.");
  return BT::NodeStatus::FAILURE; 
}

void WaitForGoalNav::callbackPoseNav(const msgPose::SharedPtr msg){
    flag_pose_recieved_ = true;
    nav_goal_ = *msg;
}


