#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"

#include <functional>
#include <limits>
#include <unistd.h> 
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

using msgPose = geometry_msgs::msg::Pose;

class WaitForGoalNav : public BT::SyncActionNode
{
public:

    WaitForGoalNav(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<msgPose>("por_out_goal_nav"),
            BT::InputPort<int>("port_int_nav_goal_timeout_sec")
        };
    }

    virtual BT::NodeStatus tick() override;
    void callbackPoseNav(const msgPose::SharedPtr);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<msgPose>::SharedPtr subPoseNav_;

    msgPose nav_goal_;
    bool flag_pose_recieved_;
    void init();
};