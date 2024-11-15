#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rb1_interfaces/srv/find_object.hpp"
#include "rb1_utils/transform_utils.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/static_transform_broadcaster.h"



#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"

#include <functional>
#include <limits>
#include <unistd.h> 
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

using srvFindObject = rb1_interfaces::srv::FindObject;

using msgPoint = geometry_msgs::msg::Point;
using msgPose = geometry_msgs::msg::Pose;
using msgTransform = geometry_msgs::msg::TransformStamped;

class PublishTransform : public BT::SyncActionNode
{
public:

    PublishTransform(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Point>("port_int_position_object"),
            BT::InputPort<bool>("port_int_get_target_nav"),
            //BT::InputPort<bool>("publish_posisition_shelf"),

            BT::OutputPort<msgPose>("port_out_target_nav_goal"),
        };
    }

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster_;

    std::string frame_child_;
    std::string frame_parent_;

    // tf utils
    std::shared_ptr<TransformUtils> tfUtils_;

    void init();
    void initParameters();
    void clearVariables();

    msgTransform changeTransform(float, float);
    msgPose sendTargetNavGoal(msgTransform&);

    // params
    double distance_aproach_target_shelf_;
    double radius_rb1_;
    double large_shelf_;
};