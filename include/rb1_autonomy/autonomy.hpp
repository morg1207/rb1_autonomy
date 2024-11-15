
#pragma once

#include "rb1_autonomy/plugins_bt/client_find_object.hpp"
#include "rb1_autonomy/plugins_bt/client_approach_shelf.hpp"
#include "rb1_autonomy/plugins_bt/client_init_localziation.hpp"
#include "rb1_autonomy/plugins_bt/client_nav.hpp"

#include "rb1_autonomy/plugins_bt/nav_poses.hpp"
#include "rb1_autonomy/plugins_bt/publish_transform.hpp"
#include "rb1_autonomy/plugins_bt/handler_platform.hpp"
#include "rb1_autonomy/plugins_bt/change_footprint.hpp"
#include "rb1_autonomy/plugins_bt/turn_robot.hpp"
#include "rb1_autonomy/plugins_bt/wait_for_goal_nav.hpp"
#include "rb1_autonomy/plugins_bt/check_approach.hpp"


#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace BT;

using msgString = std_msgs::msg::String;

class AutonomyRb1 : public rclcpp::Node {

public:
  AutonomyRb1();
  void registerNodes();

private:
  BT::BehaviorTreeFactory factory;
  BT::Tree tree_;

  rclcpp::Subscription<msgString>::SharedPtr subBtSelector_;
  rclcpp::Publisher<msgString>::SharedPtr pubStatusTask_;

  std::vector<std::string> valid_bt_select_;

  std::string package_name_;
  std::string bt_xml_;
  std::string full_bt_xml_path_;
  std::string package_share_;

  void init();

  std::string convertString(const std::string &);

  void callbackBtSelector(const msgString &);
  bool checkBtSelect(std::string &);
  void createBt(std::string&);
  bool tickTree();
};
