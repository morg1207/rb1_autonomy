#include "rb1_autonomy/plugins_bt/check_approach.hpp"

CheckApproach::CheckApproach(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {
        std::cout << "initialized [bt_check_approach]"<<std::endl;

    }

BT::PortsList CheckApproach::providedPorts()
{
  return { BT::InputPort<std::string>("port_int_control_state") };
}

BT::NodeStatus CheckApproach::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckApproach::onRunning()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::string control_state;

  if (!getInput<std::string>("port_int_control_state", control_state))
  {
    throw BT::RuntimeError("Missing required input [port_int_control_state]");
  }

  if (control_state == "END")
  {
    std::cout << "[MoveBase: FINISHED]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void CheckApproach::onHalted()
{
  std::cout << "[MoveBase: ABORTED]" << std::endl;
}
