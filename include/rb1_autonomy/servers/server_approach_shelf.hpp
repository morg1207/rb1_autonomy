#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "rb1_interfaces/srv/approach_shelf.hpp"
#include "rb1_utils/transform_utils.hpp"

#include "vector"
#include "iostream"
#include "functional"
#include "chrono"

using namespace std::chrono_literals;
using namespace std::placeholders;

using msgPoint = geometry_msgs::msg::Point;
using msgTwist = geometry_msgs::msg::Twist;
using msgTransform = geometry_msgs::msg::TransformStamped;

using srvApproachShelf = rb1_interfaces::srv::ApproachShelf;


enum class ControlState {
  INIT,
  CONTROL_APPROACH,
  CONTROL_DIRECTION1,
  CONTROL_DIRECTION2,
  CONTROL_SHELF,
  CONTROL_APPROACH_END,
  ENTER_TO_SHELF,
  BACK_TO_SHELF,
  END,
};


class ServerApproachShelf : public rclcpp::Node{

public:
    ServerApproachShelf();

    void callbackApproachShelf(const std::shared_ptr<srvApproachShelf::Request> , const std::shared_ptr<srvApproachShelf::Response>);

private:
    // tf utils
    std::shared_ptr<TransformUtils> tfUtils_;
    //
    rclcpp::Service<srvApproachShelf>::SharedPtr srvApproach_;
    rclcpp::Publisher<msgTwist>::SharedPtr pubCmdVel_;

    //tf
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    void init();
    void initParameters();
    void clearVariables();

    void robotStop();
    float satureVel(float, float, float );
    void controlRobot();
    void calculateErrors(msgTransform&);

    // control variables
    float distance_target_; 
    float theta_target_;
    double yaw_target_;

    std::string type_control_;
    ControlState control_state_;

    // params
    float distance_approach_target_error_;
    float angle_approach_target_error_;
    float vel_min_linear_x_;
    float vel_min_angular_z_;
    float vel_max_linear_x_;
    float vel_max_angular_z_;
    float kp_angular_;
    float kp_lineal_;
    float laser_min_range_;
    float distance_approach_target_error_back_;
    float distance_for_back_frame_publish_;

};