#include "rb1_autonomy/plugins_bt/client_init_localziation.hpp"


ClientInitLocalization::ClientInitLocalization(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){

  name_service = "server_init_localization";
  node_ = rclcpp::Node::make_shared("bt_client_init_localization");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_client_init_localization]");
  clientInitLocalization_ = node_->create_client<srvInitLoc>(name_service);

}

BT::NodeStatus ClientInitLocalization::tick(){

  msgPoint position_charge_station;

  if (!getInput<msgPoint>("port_int_position_charge_station", position_charge_station)) {

      throw BT::RuntimeError("missing required input [port_int_position_charge_station");
  }

  while (!clientInitLocalization_->wait_for_service(std::chrono::seconds(1))) {

    if (!rclcpp::ok()){
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service [%s]",name_service.c_str() );
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service [%s] to appear...", name_service.c_str());
  }

  auto node_status = callService(position_charge_station);  

  return node_status;
}

BT::NodeStatus ClientInitLocalization::callService(msgPoint& pose_charge_station){

  auto request = std::make_shared<srvInitLoc::Request>();

  request->station_position = pose_charge_station ;

  auto result_future = clientInitLocalization_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {

    auto response = result_future.get();

    if(response->success){

      return BT::NodeStatus::SUCCESS;
    }
    else{

      return BT::NodeStatus::FAILURE;
    }
  }
  else{

    RCLCPP_ERROR(node_->get_logger(), "Unable to call [%s]",name_service.c_str() );

    return BT::NodeStatus::FAILURE;
  }
}
