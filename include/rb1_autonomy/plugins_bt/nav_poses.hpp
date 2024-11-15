#pragma once

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"

#include <functional>
#include <limits>
#include <unistd.h> 
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

using msgString = std_msgs::msg::String;
using msgPoseNav = geometry_msgs::msg::Pose;

class NavPoses : public BT::SyncActionNode
{
public:

    NavPoses(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<msgPoseNav>("port_out_target_nav_goal"),
            BT::InputPort<std::string>("port_int_nav_poses")
        };
    }

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<msgString>::SharedPtr pubElevatorUp_;
    rclcpp::Publisher<msgString>::SharedPtr pubElevatorDown_;

    std::vector<double> nav_poses_x_;
    std::vector<double> nav_poses_y_;
    std::vector<double> nav_poses_z_;

    void init();
    void initParameters();
};