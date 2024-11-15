#pragma once

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"


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

class HandlerPlatform : public BT::SyncActionNode
{
public:

    HandlerPlatform(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("port_int_type_action")
        };
    }

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<msgString>::SharedPtr pubElevatorUp_;
    rclcpp::Publisher<msgString>::SharedPtr pubElevatorDown_;


    void init();
};