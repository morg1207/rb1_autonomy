#pragma once

#include "rclcpp/rclcpp.hpp"

#include "rb1_interfaces/srv/init_localization.hpp"
#include "rb1_utils/transform_utils.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "functional"
#include "chrono"
#include "memory"

using namespace std::chrono_literals;
using namespace std::placeholders;



using srvInitLoc = rb1_interfaces::srv::InitLocalization;
using msgPoint = geometry_msgs::msg::Point;
using msgTransform = geometry_msgs::msg::TransformStamped;
using msgPoseCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;

using namespace std::chrono_literals;


class ServerInitLocalization : public rclcpp::Node{

public:

    ServerInitLocalization();

    void callbackInitLocalization(const std::shared_ptr<srvInitLoc::Request> , const std::shared_ptr<srvInitLoc::Response>);

private:

    std::shared_ptr<TransformUtils> tfUtils_;

    rclcpp::Service<srvInitLoc>::SharedPtr srvInitLoc_;
    rclcpp::Publisher<msgPoseCovariance>::SharedPtr pubInitialPose;
    
    //tf
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    void init();
    void initParameters();
    void clearVariables();
    void publishInitialPose(msgPoint&);
    msgPoint findInitialPose(msgTransform&);
    // params
    double pose_map_to_station_charge_x_;
    double pose_map_to_station_charge_y_;
    double pose_map_to_station_charge_yaw_;

};

