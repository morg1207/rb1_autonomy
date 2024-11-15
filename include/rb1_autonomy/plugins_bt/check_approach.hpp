#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>
#include <chrono>
#include <thread>

class CheckApproach : public BT::StatefulActionNode
{
public:
    CheckApproach(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};
