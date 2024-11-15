#include "rb1_autonomy/plugins_bt/client_approach_shelf.hpp"


ClientApproachShelf::ClientApproachShelf(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){
  
  
  name_service = "server_approach_shelf";
  node_ = rclcpp::Node::make_shared("bt_client_approach_shelf");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_client_approach_shelf]");

  clientApproachShelf_ = node_->create_client<srvApproachShelf>(name_service);

}

BT::NodeStatus ClientApproachShelf::tick(){

  std::string type_control;

  if (!getInput<std::string>("port_int_type_control", type_control)) {

      throw BT::RuntimeError("missing required input [port_int_type_object");
  }

  while (!clientApproachShelf_->wait_for_service(std::chrono::seconds(1))) {

    if (!rclcpp::ok()){
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service [%s]",name_service.c_str() );
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service [%s] to appear...", name_service.c_str());
  }

  auto node_status = callService(type_control);  

  return node_status;
}

BT::NodeStatus ClientApproachShelf::callService(std::string& type_control){

  auto request = std::make_shared<srvApproachShelf::Request>();

  request->type_control = type_control ;

  auto result_future = clientApproachShelf_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {

    auto response = result_future.get();

    if(response->success){

      RCLCPP_INFO(node_->get_logger(), "Current Control State: [%s]", response->state_control_output.c_str());
      setOutput<std::string>("port_out_control_state_output",response->state_control_output);
      
      return BT::NodeStatus::SUCCESS;
    }
    else{

      RCLCPP_WARN(node_->get_logger(), "Control of type [%s] was not performed", type_control.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
  else{

    RCLCPP_ERROR(node_->get_logger(), "Unable to call [%s]",name_service.c_str() );

    return BT::NodeStatus::FAILURE;
  }
}
