#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <functional>
#include <limits>
#include <unistd.h> 
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

using msgTwist = geometry_msgs::msg::Twist;
using msgOdom = nav_msgs::msg::Odometry;

class TurnRobot : public BT::StatefulActionNode
{
public:

    TurnRobot(const std::string& , const BT::NodeConfiguration&);
 
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>("port_int_angle_rotate")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<msgTwist>::SharedPtr pubCmdVel_;
    rclcpp::Subscription<msgOdom>::SharedPtr subOdom_;

    void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr);

    msgTwist msgCmdVel;
    
    void init();
    void initParameters();

    double calculateAngleRotated(double, double);

    double yaw_;
    double yaw_last_;
    double yaw_init_;

    bool flag_odom_;

    float angle_rotate_;

    float count_angle_rotate_;
    //params
    float vel_turn_;
};