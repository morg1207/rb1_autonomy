#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"

#include <functional>
#include <limits>
#include <unistd.h> 
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

using msgPolygon = geometry_msgs::msg::Polygon;
using msgPoint32 = geometry_msgs::msg::Point32;

class ChangeFootprint : public BT::SyncActionNode
{
public:

    ChangeFootprint(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("port_int_type_change")
        };
    }

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<msgPolygon>::SharedPtr pubChangeFootprintLocal_;
    rclcpp::Publisher<msgPolygon>::SharedPtr pubChangeFootprintGlobal_;

    void init();
    void initParameters();

    double shelf_length_;
    double shelf_width_;
    double rb1_radius_;
};