#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rb1_interfaces/srv/init_localization.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/basic_types.h"

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

using namespace std::placeholders;

using srvInitLoc = rb1_interfaces::srv::InitLocalization;
using msgPoint = geometry_msgs::msg::Point;

class ClientInitLocalization : public BT::SyncActionNode
{
public:

    ClientInitLocalization(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts(){
        return{ 
            BT::InputPort<msgPoint>("port_int_position_charge_station"),
        };
    }

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<srvInitLoc>::SharedPtr clientInitLocalization_;
    
    BT::NodeStatus callService(msgPoint &);
    std::string name_service;
};