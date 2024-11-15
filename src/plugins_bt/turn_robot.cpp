#include "rb1_autonomy/plugins_bt/turn_robot.hpp"


TurnRobot::TurnRobot(const std::string& name, const BT::NodeConfiguration& config): BT::StatefulActionNode(name, config){


  node_ = rclcpp::Node::make_shared("bt_turn_robot");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_turn_robot]");
  init();
  initParameters();
}

void TurnRobot::init(){

  pubCmdVel_ = node_->create_publisher<msgTwist>("cmd_vel",10);
  subOdom_ = node_->create_subscription<msgOdom>("odom",10,std::bind(&TurnRobot::callbackOdom,this,_1));

}
void TurnRobot::initParameters(){

  node_->declare_parameter("vel_turn", 0.4);
  vel_turn_ = node_->get_parameter("vel_turn").as_double();
  RCLCPP_INFO(node_->get_logger(), "vel_turn [%.3f] ", vel_turn_);

}

void TurnRobot::callbackOdom(const nav_msgs::msg::Odometry::SharedPtr odom) {

  RCLCPP_DEBUG(node_->get_logger(), "Odom subscribers ");
  double roll, pitch;
    
  tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);

  flag_odom_ = true;
  if (yaw_ < 0){
    yaw_ = yaw_ + M_PI*2;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Current Yaw  [%.3f] ", yaw_);
}

BT::NodeStatus TurnRobot::onStart() {



  if (!getInput<float>("port_int_angle_rotate", angle_rotate_)) {

    throw BT::RuntimeError("missing required input [port_int_angle_rotate]");
  }

  count_angle_rotate_ = 0.0;
   
  RCLCPP_INFO(node_->get_logger(), "Rotating the robot %.3f", angle_rotate_);

  rclcpp::Rate rate(20);

  while (flag_odom_ != true) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }
  yaw_init_ = yaw_;
  yaw_last_ = yaw_init_;

  RCLCPP_INFO(node_->get_logger(), "Initial yaw [%.3f] ", yaw_init_);

  msgCmdVel.angular.z = vel_turn_;
  msgCmdVel.linear.x = 0.0;
  pubCmdVel_->publish(msgCmdVel);

  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus TurnRobot::onRunning() {

  rclcpp::spin_some(node_);

  if( flag_odom_){
    float desp_angle = yaw_ - yaw_last_ ;

    if(desp_angle<0 && abs(desp_angle) > 0.0001){
        desp_angle = desp_angle + 2*M_PI;
    }
    count_angle_rotate_ = desp_angle + count_angle_rotate_;
    RCLCPP_INFO(node_->get_logger(), "Rotated angle: %.3f", count_angle_rotate_);

    yaw_last_ = yaw_;
    flag_odom_ = false;
  }

  if( count_angle_rotate_ < angle_rotate_){

    msgCmdVel.angular.z = vel_turn_;
    msgCmdVel.linear.x = 0.0;
    pubCmdVel_->publish(msgCmdVel);

    return BT::NodeStatus::RUNNING;
  }
  else{

    msgCmdVel.angular.z = 0.0;
    msgCmdVel.linear.x = 0.0;
    pubCmdVel_->publish(msgCmdVel);

    return BT::NodeStatus::SUCCESS;
  }

}

double TurnRobot::calculateAngleRotated(double initial_yaw, double current_yaw) {

    return 0;
}

void TurnRobot::onHalted() { printf("[ MoveBase: ABORTED ]"); }

