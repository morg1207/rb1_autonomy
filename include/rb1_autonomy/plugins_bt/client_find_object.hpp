#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rb1_interfaces/srv/find_object.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/basic_types.h"

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

using namespace std::placeholders;

using srvFindObject = rb1_interfaces::srv::FindObject;

class ClientFindObject : public BT::SyncActionNode
{
public:

    ClientFindObject(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts(){
        return{ 
            BT::OutputPort<geometry_msgs::msg::Point>("port_out_position_object"),
            BT::InputPort<std::string>("port_int_type_object"),
        };
    }

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<srvFindObject>::SharedPtr clientFindObject_;
    
    BT::NodeStatus callService(std::string &);
    std::string name_service;
};