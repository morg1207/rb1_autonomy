#include "rb1_autonomy/servers/server_approach_shelf.hpp"


ServerApproachShelf::ServerApproachShelf() : Node("server_approach_shelf"){

  RCLCPP_INFO(get_logger(), "Node initialized [server_approach_shelf]");

  init();
  initParameters();
}

void ServerApproachShelf::init(){

  control_state_ = ControlState::INIT;

  RCLCPP_INFO(get_logger(), "Service server [approach_shelf_server] initialized");
  
  pubCmdVel_ = create_publisher<msgTwist>("cmd_vel",10);
  //------------------------------------
  srvApproach_ = create_service<srvApproachShelf>("server_approach_shelf",std::bind(&ServerApproachShelf::callbackApproachShelf,this,_1,_2));
  
  tfBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

}
void ServerApproachShelf::callbackApproachShelf(const std::shared_ptr<srvApproachShelf::Request> request, const std::shared_ptr<srvApproachShelf::Response> response){

  // Transform utils
  tfUtils_ = std::make_shared<TransformUtils>(shared_from_this());

  std::string to_frame = "robot_base_link";
  std::string from_frame = "robot_front_laser_base_link";

  msgTransform t = tfUtils_->getTransform(tfBuffer_,to_frame,from_frame);
  type_control_ = request->type_control;
  controlRobot();

  if (control_state_ == ControlState::END) {

    response->state_control_output = "END";
    control_state_ = ControlState::INIT;
  } 
  else {

    response->state_control_output = "RUNING";
  }
  response->success = true;
}


void ServerApproachShelf::calculateErrors(msgTransform& t){

  RCLCPP_DEBUG(get_logger(), " ");
  RCLCPP_DEBUG(get_logger(), "**************** Errors *****************");
  RCLCPP_DEBUG(get_logger(), " ");

  distance_target_ = sqrt(std::pow(t.transform.translation.x, 2) + 
      std::pow(t.transform.translation.y, 2));

  theta_target_ = std::atan2(t.transform.translation.y, t.transform.translation.x);

  double roll, pitch;
  // Hallar el angulo target de direccion
  tf2::Quaternion quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_target_);
  
  RCLCPP_DEBUG(get_logger(), "Distance [%.3f], theta_target [%.3f], yaw_target [%.3f]", distance_target_, theta_target_, yaw_target_);
}

void ServerApproachShelf::controlRobot() {

  RCLCPP_INFO(get_logger(), "Control type [%s]", type_control_.c_str());

  msgTransform t;
  msgTwist msg_cmd_vel;

  if (type_control_ == "approach_shelf") {

    switch (control_state_) {
    case ControlState::INIT:

      RCLCPP_DEBUG(get_logger(), "Control state [CONTROL_INIT]");
      control_state_ = ControlState::CONTROL_DIRECTION1;

      break;

    case ControlState::CONTROL_DIRECTION1:

      t = tfUtils_->getTransform(tfBuffer_,"robot_base_link","target_front_frame");
      calculateErrors(t);

      
      if (abs(theta_target_) < angle_approach_target_error_) {
        control_state_ = ControlState::CONTROL_APPROACH;
        robotStop();
      }

      RCLCPP_DEBUG(get_logger(), "Control state [DIRECCTION1]");
        
      msg_cmd_vel.angular.z = theta_target_ * kp_angular_;
      msg_cmd_vel.linear.x = 0.0;

      msg_cmd_vel.angular.z = satureVel(msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);

      RCLCPP_DEBUG(get_logger(), "Set Linear Velocity x [%.3f]  Angular Velocity z [%.3f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
      pubCmdVel_->publish(msg_cmd_vel);

      break;

    case ControlState::CONTROL_APPROACH:

      t = tfUtils_->getTransform(tfBuffer_,"robot_base_link","target_front_frame");

      calculateErrors(t);

      if (distance_target_ < distance_approach_target_error_) {

        control_state_ = ControlState::CONTROL_DIRECTION2;
        robotStop();
      }

      RCLCPP_DEBUG(get_logger(), "Control State [CONTROL_APPROACH]");

      msg_cmd_vel.angular.z = theta_target_ * kp_angular_;
      msg_cmd_vel.linear.x = distance_target_ * kp_lineal_;

      msg_cmd_vel.linear.x = satureVel(msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);
      msg_cmd_vel.angular.z = satureVel(msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
      RCLCPP_DEBUG(get_logger(), "Set Vel x [%.3f]  Velz [%.3f]",msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
      pubCmdVel_->publish(msg_cmd_vel);

      break;

    case ControlState::CONTROL_DIRECTION2:

      t = tfUtils_->getTransform(tfBuffer_,"robot_base_link","target_front_frame");

      calculateErrors(t);
      if (abs(yaw_target_) < angle_approach_target_error_) {

        control_state_ = ControlState::CONTROL_APPROACH_END;
        //control_state_ = ControlState::END;
        robotStop();
      }
      RCLCPP_DEBUG(get_logger(), "Control State [DIRECTION2]");
      msg_cmd_vel.angular.z = yaw_target_ * kp_angular_;
      msg_cmd_vel.linear.x = 0.0;

      msg_cmd_vel.angular.z = satureVel(msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
      RCLCPP_DEBUG(get_logger(), "Set Linear Velocity x [%.3f]  Angular Velocity z [%.3f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
      pubCmdVel_->publish(msg_cmd_vel);

      break;

    case ControlState::CONTROL_APPROACH_END:

      t = tfUtils_->getTransform(tfBuffer_,"robot_base_link","shelf_frame");

      calculateErrors(t);
      if (distance_target_ < laser_min_range_) {

        control_state_ = ControlState::END;
        robotStop();
       }
      RCLCPP_DEBUG(get_logger(), "Control State [CONTROL_APPROACH_END]");
      msg_cmd_vel.angular.z = theta_target_ * kp_angular_;
      msg_cmd_vel.linear.x = distance_target_ * kp_lineal_;
      msg_cmd_vel.linear.x = satureVel(msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);
      msg_cmd_vel.angular.z = satureVel(msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
      RCLCPP_DEBUG(get_logger(), "Set Vel x [%.3f]  Velz [%.3f]",msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
      pubCmdVel_->publish(msg_cmd_vel);

      break;

    default:
      RCLCPP_DEBUG(get_logger(), "State default");
      control_state_ = ControlState::INIT;
      break;
    }
  } 
  else if (type_control_ == "enter_to_shelf") {

    switch (control_state_) {

    case ControlState::INIT:

      RCLCPP_DEBUG(get_logger(), "Control State [CONTROL_INIT]");
      control_state_ = ControlState::ENTER_TO_SHELF;
      
      break;

    case ControlState::ENTER_TO_SHELF:

      t = tfUtils_->getTransform(tfBuffer_,"robot_base_link","shelf_center_frame");

      calculateErrors(t);

      if (distance_target_ < distance_approach_target_error_) {

        control_state_ = ControlState::END;
        robotStop();
      }
      RCLCPP_DEBUG(get_logger(), "Target Distance [%.3f]", distance_target_);
      RCLCPP_DEBUG(get_logger(), "Control State [ENTER_TO_SHELF]");
      msg_cmd_vel.angular.z = theta_target_ * kp_angular_;
      msg_cmd_vel.linear.x = distance_target_ * kp_lineal_;
      msg_cmd_vel.linear.x = satureVel(
            msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);
      msg_cmd_vel.angular.z = satureVel(
            msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
      RCLCPP_DEBUG(get_logger(), "Set Linear Velocity x [%.3f] and Angular Velocity z [%.3f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
      pubCmdVel_->publish(msg_cmd_vel);

      break;

    default:

      control_state_ = ControlState::INIT;

      break;
      }
    } 

  else if (type_control_ == "back_to_shelf") {

    geometry_msgs::msg::Point position_object;
    std::string frame_parent_ = "robot_odom";
    std::string frame_child_ = "shelf_back_frame";

    switch (control_state_) {

    case ControlState::INIT:

      position_object.x = distance_for_back_frame_publish_;
      position_object.y = M_PI;
      position_object.z = 0.0;

      t = tfUtils_->getTransform(tfBuffer_,"robot_odom","robot_front_laser_base_link");

      t = tfUtils_->findTransform(t,position_object);

      tfUtils_->sendTransform(tfBroadcaster_,t,frame_child_, frame_parent_);

      control_state_ = ControlState::BACK_TO_SHELF;
      RCLCPP_DEBUG(get_logger(), "Control State [CONTROL_INIT]");
      break;

    case ControlState::BACK_TO_SHELF:

      t = tfUtils_->getTransform(tfBuffer_,"robot_base_link","shelf_back_frame");

      calculateErrors(t);
      if (distance_target_ < distance_approach_target_error_back_) {
        // if (distance_target_ <) {
        control_state_ = ControlState::END;
        robotStop();
      }
      if (theta_target_ > 0) {
        theta_target_ = theta_target_ - M_PI;
      } 
      else {
        theta_target_ = M_PI - theta_target_;
      }
      RCLCPP_DEBUG(get_logger(), "Distance error [%.3f]", distance_target_);
      RCLCPP_DEBUG(get_logger(), "Target theta [%.3f]", theta_target_);
      RCLCPP_DEBUG(get_logger(), "Control state [BACK_TO_SHELF]");

      msg_cmd_vel.angular.z = 0.0;
      msg_cmd_vel.linear.x = distance_target_ * kp_lineal_;

      msg_cmd_vel.linear.x = satureVel(msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);

      // retroceso
      msg_cmd_vel.linear.x = - msg_cmd_vel.linear.x;
      RCLCPP_DEBUG(get_logger(), "Set velocity x [%.3f]  angular velocity z [%.3f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
      pubCmdVel_->publish(msg_cmd_vel);

      break;

    default:
        control_state_ = ControlState::INIT;
      break;
    }
  }
}

float ServerApproachShelf::satureVel(float vel, float vel_lower_limit, float vel_upper_limit){

  if (vel > 0) {
    return std::max(vel_lower_limit, std::min(vel, vel_upper_limit));
  } else {
    vel = std::max(vel_lower_limit, std::min(-vel, vel_upper_limit));
    return -vel;
  }

}

void ServerApproachShelf::robotStop() {
  msgTwist msg_cmd_vel;
  msg_cmd_vel.angular.z = 0.0;
  msg_cmd_vel.linear.x = 0.0;
  pubCmdVel_->publish(msg_cmd_vel);
}

void ServerApproachShelf::initParameters(){

  // Declare parameter for target approach distance error and get its value
  declare_parameter("distance_approach_target_error", 0.5);
  distance_approach_target_error_ = get_parameter("distance_approach_target_error").as_double();
  RCLCPP_INFO(get_logger(), "Target approach distance error [%.3f]", distance_approach_target_error_);

  // Declare parameter for back approach distance error and get its value
  declare_parameter("distance_approach_target_error_back", 0.05);
  distance_approach_target_error_back_ = get_parameter("distance_approach_target_error_back").as_double();
  RCLCPP_INFO(get_logger(), "Back target approach distance error [%.3f]", distance_approach_target_error_back_);

  // Declare parameter for angle approach target error and get its value
  declare_parameter("angle_approach_target_error", 0.5);
  angle_approach_target_error_ = get_parameter("angle_approach_target_error").as_double();
  RCLCPP_INFO(get_logger(), "Target approach angle error [%.3f]", angle_approach_target_error_);

  // Declare parameter for minimum linear velocity in the x direction and get its value
  declare_parameter("vel_min_linear_x", 0.5);
  vel_min_linear_x_ = get_parameter("vel_min_linear_x").as_double();
  RCLCPP_INFO(get_logger(), "Minimum linear velocity x [%.3f]", vel_min_linear_x_);

  // Declare parameter for minimum angular velocity in the z direction and get its value
  declare_parameter("vel_min_angular_z", 0.5);
  vel_min_angular_z_ = get_parameter("vel_min_angular_z").as_double();
  RCLCPP_INFO(get_logger(), "Minimum angular velocity z [%.3f]", vel_min_angular_z_);

  // Declare parameter for maximum linear velocity in the x direction and get its value
  declare_parameter("vel_max_linear_x", 0.5);
  vel_max_linear_x_ = get_parameter("vel_max_linear_x").as_double();
  RCLCPP_INFO(get_logger(), "Maximum linear velocity x [%.3f]", vel_max_linear_x_);

  // Declare parameter for maximum angular velocity in the z direction and get its value
  declare_parameter("vel_max_angular_z", 0.5);
  vel_max_angular_z_ = get_parameter("vel_max_angular_z").as_double();
  RCLCPP_INFO(get_logger(), "Maximum angular velocity z [%.3f]", vel_max_angular_z_);

  // Declare parameter for angular proportional gain and get its value
  declare_parameter("kp_angular", 0.5);
  kp_angular_ = get_parameter("kp_angular").as_double();
  RCLCPP_INFO(get_logger(), "Angular proportional gain Kp [%.3f]", kp_angular_);

  // Declare parameter for linear proportional gain and get its value
  declare_parameter("kp_lineal", 0.5);
  kp_lineal_ = get_parameter("kp_lineal").as_double();
  RCLCPP_INFO(get_logger(), "Linear proportional gain Kp [%.3f]", kp_lineal_);

  // Declare parameter for laser minimum range and get its value
  declare_parameter("laser_min_range", 0.5);
  laser_min_range_ = get_parameter("laser_min_range").as_double();
  RCLCPP_INFO(get_logger(), "Laser minimum range [%.3f]", laser_min_range_);

  // Declare parameter for distance to back frame publish and get its value
  declare_parameter("distance_for_back_frame_publish", 0.5);
  distance_for_back_frame_publish_ = get_parameter("distance_for_back_frame_publish").as_double();
  RCLCPP_INFO(get_logger(), "Distance for back frame publish [%.3f]", distance_for_back_frame_publish_);

}