#include "rb1_autonomy/plugins_bt/client_nav.hpp"

using namespace std::placeholders;

ClientNav::ClientNav(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<NavToPose>(name, conf, params), cancel_nav_(false)
{
  node_ = rclcpp::Node::make_shared("bt_client_nav");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_client_nav]");
  subCancel_ = node_->create_subscription<std_msgs::msg::Bool>(
      "cancel_nav", 10, std::bind(&ClientNav::callbackCancel, this, _1));
}

BT::PortsList ClientNav::providedPorts()
{
  return providedBasicPorts({InputPort<msgPose>("goal_nav")});
}

bool ClientNav::setGoal(RosActionNode::Goal &goal)
{
  msgPose goal_nav;
  if (!getInput<msgPose>("goal_nav", goal_nav))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get input goal_nav");
    return false;
  }

  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose = goal_nav;

  RCLCPP_INFO(node_->get_logger(), "The goal was sent");
  return true;
}

BT::NodeStatus ClientNav::onResultReceived(const WrappedResult &wr)
{
  RCLCPP_INFO(node_->get_logger(), "Navigation completed successfully");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ClientNav::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ClientNav::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  if (cancel_nav_)
  {
    RCLCPP_INFO(node_->get_logger(), "Navigation canceled");
    return BT::NodeStatus::SUCCESS;
  }

  const auto &current_position = feedback->current_pose.pose.position;
  RCLCPP_INFO(node_->get_logger(), "Current position x = [%.3f], y = [%.3f]", current_position.x, current_position.y);
  rclcpp::spin_some(node_);

  return BT::NodeStatus::RUNNING;
}

void ClientNav::callbackCancel(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Data to cancel has arrived");
  cancel_nav_ = msg->data;
}
