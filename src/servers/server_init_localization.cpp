#include "rb1_autonomy/servers/server_init_localization.hpp"


ServerInitLocalization::ServerInitLocalization() : Node("server_init_localization"){

  RCLCPP_INFO(get_logger(), "Node initialized [server_init_localization]");
  init();
  initParameters();
}

void ServerInitLocalization::init(){
  
  RCLCPP_INFO(get_logger(), "Service server [server_init_localization] initialized");

  tfBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  srvInitLoc_ = create_service<srvInitLoc>("server_init_localization", std::bind(&ServerInitLocalization::callbackInitLocalization,this,_1,_2));

  pubInitialPose = create_publisher<msgPoseCovariance>("initialpose",10);

}


void ServerInitLocalization::callbackInitLocalization(const std::shared_ptr<srvInitLoc::Request> request, const std::shared_ptr<srvInitLoc::Response> response){

  clearVariables();

  /*

  if (auto shared_this = weak_from_this().lock()) {
        
  } else {
        RCLCPP_ERROR(get_logger(), "Failed to create shared pointer in constructor.");
  }
  */
  tfUtils_ = std::make_shared<TransformUtils>(shared_from_this());

  msgPoint pose_charge_station = request->station_position;

  RCLCPP_DEBUG(get_logger(),"-------------------------------------------");
  RCLCPP_DEBUG(get_logger(), "Station position x [%.3f] y [%.3f] yaw [%.3f]" , pose_charge_station.x, pose_charge_station.y, pose_charge_station.z);

  // get transform
  std::string to_frame = "robot_base_link";
  std::string from_frame = "robot_front_laser_base_link";

  msgTransform t = tfUtils_->getTransform(tfBuffer_,to_frame,from_frame);

  // find transform
  t = tfUtils_->findTransform(t,pose_charge_station);

  // send tranform
  std::string child_frame = "charge_station";
  std::string parend_frame = "robot_base_link";
  tfUtils_->sendTransform(tfBroadcaster_,t,child_frame, parend_frame);

  // find initial pose
  msgPoint pose = findInitialPose(t);
  publishInitialPose(pose);

  response->success = true;
}



msgPoint ServerInitLocalization::findInitialPose(msgTransform& msg_transform){

  // hallo el angulo de transformacion
  float pose_station_x = msg_transform.transform.translation.x;
  float pose_station_y = msg_transform.transform.translation.y;

  double roll, pitch, yaw;
  tf2::Quaternion quaternion(msg_transform.transform.rotation.x, msg_transform.transform.rotation.y,
                                 msg_transform.transform.rotation.z, msg_transform.transform.rotation.w);

  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  float pose_station_yaw = yaw;

  RCLCPP_DEBUG(get_logger(),
             "Yaw of the frame [charge_station] is [%.3f]!!!",
             pose_station_yaw);

  TransformationMatrix2D matrix = tfUtils_->createTransformationMatrix(
        pose_station_x, pose_station_y, pose_station_yaw);

  std::cout << "2D transformation matrix:" << std::endl;
  matrix.print();

  TransformationMatrix2D inverseMatrix = tfUtils_->invertTransformationMatrix(matrix);

  // subtract the vectors robot - map
  float theta_robot = std::atan2(inverseMatrix.m[1][2], inverseMatrix.m[0][2]);
  RCLCPP_DEBUG(get_logger(), "Theta robot [%.3f]!!!", theta_robot);
  float theta_map = std::atan2(pose_map_to_station_charge_y_, pose_map_to_station_charge_x_);
  RCLCPP_DEBUG(get_logger(), "Theta map [%.3f]!!!", theta_map);
  float distance_robot = std::sqrt(std::pow(pose_station_x, 2) + std::pow(pose_station_y, 2));
  RCLCPP_DEBUG(get_logger(), "Distance robot [%.3f]!!!", distance_robot);
  float distance_map = std::sqrt(std::pow(pose_map_to_station_charge_x_, 2) + std::pow(pose_map_to_station_charge_y_, 2));
  RCLCPP_DEBUG(get_logger(), "Distance map [%.3f]!!!", distance_map);
  float theta_final = theta_robot - theta_map;
  RCLCPP_DEBUG(get_logger(), "Theta final[%.3f]!!!", theta_final);
  float distance_final = std::sqrt(std::pow(distance_robot, 2) + std::pow(distance_map, 2) -
                    2 * distance_robot * distance_map * std::cos(theta_final));
  RCLCPP_DEBUG(get_logger(), "Final distance [%.3f]!!!", distance_final);
  // sum as vectors i and j
  float x_end = distance_robot * std::cos(theta_robot) - distance_map * std::cos(theta_map);
  float y_end = distance_robot * std::sin(theta_robot) - distance_map * std::sin(theta_map);
  // theta
  float theta_direction_charge_robot = std::atan2(y_end, x_end);
  RCLCPP_DEBUG(get_logger(), "Final direction theta [%.3f]!!!", theta_direction_charge_robot);
  // theta robot_map
  float theta_charge_robot = std::atan2(inverseMatrix.m[1][0], inverseMatrix.m[0][0]);
  RCLCPP_DEBUG(get_logger(), "Theta robot [%.3f]!!!", theta_charge_robot);
  // Angle of map robot base link
  float theta_direction_robot_map = theta_direction_charge_robot - M_PI_2;
  float theta_robot_map = theta_charge_robot - M_PI_2;
  // pose robot to map
  float pose_robot_to_map_x = distance_final * std::cos(theta_direction_robot_map);
  float pose_robot_to_map_y = distance_final * std::sin(theta_direction_robot_map);
  float direction = theta_robot_map;
  RCLCPP_DEBUG(get_logger(), "The robot is located with respect to the map at x [%.3f] y [%.3f] theta [%.3f]",
                  pose_robot_to_map_x, pose_robot_to_map_y, direction);


  msgPoint pose;
  pose.x = pose_robot_to_map_x;
  pose.y = pose_robot_to_map_y;
  pose.z = direction;
  
  return pose;
}

void ServerInitLocalization::clearVariables(){

}

void ServerInitLocalization::publishInitialPose(msgPoint& pose ){

  geometry_msgs::msg::PoseWithCovarianceStamped msg_initial_pose;

  msg_initial_pose.header.frame_id = "map";
  msg_initial_pose.header.stamp = this->get_clock()->now();
  msg_initial_pose.pose.pose.position.x  = pose.x;
  msg_initial_pose.pose.pose.position.y  = pose.y;
  msg_initial_pose.pose.pose.position.z  = 0.0;
  tf2::Quaternion q_end;
  q_end.setRPY(0.0, 0.0, pose.z); // Sin rotación
  msg_initial_pose.pose.pose.orientation.x = q_end.x();
  msg_initial_pose.pose.pose.orientation.y = q_end.y();
  msg_initial_pose.pose.pose.orientation.z = q_end.z();
  msg_initial_pose.pose.pose.orientation.w = q_end.w();
  msg_initial_pose.pose.covariance[0] = 0.25;
  msg_initial_pose.pose.covariance[7] = 0.25;
  msg_initial_pose.pose.covariance[35] = 0.06853891909122467;
  pubInitialPose->publish(msg_initial_pose);

}


void ServerInitLocalization::initParameters(){

  declare_parameter("pose_map_to_station_charge_x", 0.5);
  pose_map_to_station_charge_x_ = get_parameter("pose_map_to_station_charge_x").as_double();
  RCLCPP_INFO(get_logger(), "pose_map_to_station_charge_x [%.3f] ", pose_map_to_station_charge_x_);
  //----------------------------------------------------------

  declare_parameter("pose_map_to_station_charge_y", 0.05);
  pose_map_to_station_charge_y_ = get_parameter("pose_map_to_station_charge_y").as_double();
  RCLCPP_INFO(get_logger(), "Distance to target_goal error [%.3f] ", pose_map_to_station_charge_y_);
  //----------------------------------------------------------

  declare_parameter("pose_map_to_station_charge_yaw", 0.5);
  pose_map_to_station_charge_yaw_ = get_parameter("pose_map_to_station_charge_yaw").as_double();
  RCLCPP_INFO(get_logger(), "Angle to target_goal error [%.3f] ", pose_map_to_station_charge_yaw_);
  //----------------------------------------------------------

}

