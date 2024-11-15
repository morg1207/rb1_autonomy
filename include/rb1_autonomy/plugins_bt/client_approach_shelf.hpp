#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rb1_interfaces/srv/approach_shelf.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/basic_types.h"

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

using namespace std::placeholders;

using srvApproachShelf = rb1_interfaces::srv::ApproachShelf;

class ClientApproachShelf : public BT::SyncActionNode
{
public:

    ClientApproachShelf(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts(){
        return{ 
            BT::OutputPort<std::string>("port_out_control_state_output"),
            BT::InputPort<std::string>("port_int_type_control"),
        };
    }

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<srvApproachShelf>::SharedPtr clientApproachShelf_;
    
    BT::NodeStatus callService(std::string &);
    std::string name_service;
};