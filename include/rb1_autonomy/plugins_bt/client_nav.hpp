#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>

using namespace BT;
using namespace std::chrono_literals;

// Para simplificar el código
using NavToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavToPose = rclcpp_action::ServerGoalHandle<NavToPose>;
using msgPose = geometry_msgs::msg::Pose;

class ClientNav : public RosActionNode<NavToPose>
{
public:
  ClientNav(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

  static PortsList providedPorts();

  bool setGoal(RosActionNode::Goal &goal) override;

  NodeStatus onResultReceived(const WrappedResult &wr) override;

  NodeStatus onFailure(ActionNodeErrorCode error) override;

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

  void callbackCancel(const std_msgs::msg::Bool::SharedPtr msg);

  void onHalt() override {};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subCancel_;
  bool cancel_nav_;
};

