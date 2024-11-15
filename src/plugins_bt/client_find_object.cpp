#include "rb1_autonomy/plugins_bt/client_find_object.hpp"


ClientFindObject::ClientFindObject(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){

  name_service = "server_find_object";
  node_ = rclcpp::Node::make_shared("bt_client_find_object");
  RCLCPP_INFO(node_->get_logger(), "Node initialized [bt_client_find_object]");
  clientFindObject_ = node_->create_client<srvFindObject>(name_service);

}

BT::NodeStatus ClientFindObject::tick(){

  std::string find_object;

  if (!getInput<std::string>("port_int_type_object", find_object)) {

      throw BT::RuntimeError("missing required input [port_int_type_object");
  }

  while (!clientFindObject_->wait_for_service(std::chrono::seconds(1))) {

    if (!rclcpp::ok()){
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service [%s]",name_service.c_str() );
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service [%s] to appear...", name_service.c_str());
  }

  auto node_status = callService(find_object);  

  return node_status;
}

BT::NodeStatus ClientFindObject::callService(std::string& find_object){

  auto request = std::make_shared<srvFindObject::Request>();

  request->find_object = find_object ;

  auto result_future = clientFindObject_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {

    auto response = result_future.get();

    if(response->success){

      RCLCPP_INFO(node_->get_logger(), "Object of type [%s] found at position: %f, %f, %f", find_object.c_str(),
              response->object_position.x, response->object_position.y, response->object_position.z);

      setOutput<geometry_msgs::msg::Point>("port_out_position_object",response->object_position);
      
      return BT::NodeStatus::SUCCESS;
    }
    else{

      RCLCPP_WARN(node_->get_logger(), "No object of type [%s] was found", find_object.c_str());

      return BT::NodeStatus::FAILURE;
    }
  }
  else{

    RCLCPP_ERROR(node_->get_logger(), "Unable to call [%s]",name_service.c_str() );

    return BT::NodeStatus::FAILURE;
  }
}
