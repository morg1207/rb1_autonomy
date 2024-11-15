#include "rb1_autonomy/plugins_bt/handler_platform.hpp"


HandlerPlatform::HandlerPlatform(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){


  node_ = rclcpp::Node::make_shared("bt_handle_elevator");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_handle_elevator]");
  init();

}

void HandlerPlatform::init(){

  pubElevatorUp_ = node_->create_publisher<msgString>("elevator_up",10);
  pubElevatorDown_ = node_->create_publisher<msgString>("elevator_down",10);

}

BT::NodeStatus HandlerPlatform::tick(){

  std::string type_action;

  if (!getInput<std::string>("port_int_type_action", type_action)) {

    throw BT::RuntimeError("missing required input [port_int_type_object");
  }
  msgString msgElevator;

  if(type_action == "up"){

    pubElevatorUp_->publish(msgElevator);
    RCLCPP_INFO(node_->get_logger(), "The action type [%s] has been executed", type_action.c_str());

    return BT::NodeStatus::SUCCESS;
  } 
  else if(type_action == "down"){

    RCLCPP_INFO(node_->get_logger(), "The action type [%s] has been executed", type_action.c_str());
    pubElevatorDown_->publish(msgElevator);

    return BT::NodeStatus::SUCCESS;
  } 
  else{ 
    RCLCPP_ERROR(node_->get_logger(), "The action type [%s] is not recognized, please choose between -up- or -down-", type_action.c_str());
    return BT::NodeStatus::FAILURE;
  } 
}




