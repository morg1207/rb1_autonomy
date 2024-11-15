#include "rb1_autonomy/autonomy.hpp"

AutonomyRb1::AutonomyRb1() : Node("autonomy") {
  RCLCPP_INFO(get_logger(), "Node initialized [autonomy]");
  init();
}

void AutonomyRb1::registerNodes(){

  factory.registerNodeType<ClientFindObject>("ClientFindObject");
  factory.registerNodeType<ClientApproachShelf>("ClientApproachShelf");
  factory.registerNodeType<ClientInitLocalization>("ClientInitLocalization");

  factory.registerNodeType<PublishTransform>("PublishTransform");
  factory.registerNodeType<HandlerPlatform>("HandlerPlatform");
  factory.registerNodeType<CheckApproach>("CheckApproach");
  factory.registerNodeType<ChangeFootprint>("ChangeFootprint");
  factory.registerNodeType<TurnRobot>("TurnRobot");
  factory.registerNodeType<NavPoses>("NavPoses");
  factory.registerNodeType<WaitForGoalNav>("WaitForGoalNav");

  //client nav use bt_ros2
  RosNodeParams params; 
  params.nh = shared_from_this();
  params.default_port_value = "navigate_to_pose";
  params.server_timeout = std::chrono::milliseconds(120000);
  factory.registerNodeType<ClientNav>("ClientNav", params);
}

void AutonomyRb1::init(){

  subBtSelector_ = create_subscription<msgString>("bt_selector", 10, std::bind(&AutonomyRb1::callbackBtSelector, this, _1));

  pubStatusTask_ = create_publisher<msgString>("bt_status", 10);
    
  package_name_ = "rb1_autonomy";

  package_share_ = ament_index_cpp::get_package_share_directory(package_name_);

  valid_bt_select_ = {
        "find_station_and_init_localization",
        "find_shelf",
        "approach_and_pick_shelf",
        "carry_and_discharge_shelf",
        "entire"
  };
  RCLCPP_DEBUG(get_logger(), "Share Package : %s", package_share_.c_str());
  RCLCPP_INFO(get_logger(),"Please select a behavior tree");
}

std::string AutonomyRb1::convertString(const std::string &input) {
  std::string result;
  bool toUpper = true; 

  for (char ch : input) {
    if (ch == '_') {
      toUpper = true;
    } 
    else 
    {
      result += toUpper ? std::toupper(ch) : ch;
      toUpper = false;
    }
  }
  return result;
}
bool AutonomyRb1::checkBtSelect(std::string& bt_select){

  if (std::find(valid_bt_select_.begin(), valid_bt_select_.end(), bt_select) != valid_bt_select_.end()) {

    RCLCPP_INFO(get_logger(), "Behavior tree selected: %s", bt_select.c_str());
    return true;
  } 
  else 
  {
    RCLCPP_WARN(get_logger(), "bt_select_ is not valid: %s", bt_select.c_str());
    return false;
  }
}
void AutonomyRb1::createBt(std::string& bt_select){
  full_bt_xml_path_ = package_share_ + "/bt/" + bt_select + ".xml";

  factory.registerBehaviorTreeFromFile(full_bt_xml_path_);
  std::string bt_select_sub_tree = "SubTree" + convertString(bt_select);
  tree_ = factory.createTree(bt_select_sub_tree);
}


void AutonomyRb1::callbackBtSelector(const msgString& msg) {

  bool status_task;
  msgString msg_status_task;

  std::string bt_select = msg.data;
  
  if(checkBtSelect(bt_select)){
    createBt(bt_select);
  }
  else
  {
    return;
  }
  // execute bt
  status_task = tickTree();
  //send status task
  if (status_task) {

    msg_status_task.data = bt_select + "/done";
    RCLCPP_INFO(get_logger(), "Behavior tree [%s] has finished successfully", bt_select.c_str());
    RCLCPP_WARN(get_logger(), "Please select another behavior tree");
  } 
  else {
    msg_status_task.data = bt_select + "/fail";
    RCLCPP_ERROR(get_logger(), "Behavior tree [%s] failed", bt_select.c_str());
    RCLCPP_WARN(get_logger(), "Please select another behavior tree");
  }
  pubStatusTask_->publish(msg_status_task);
}

bool AutonomyRb1::tickTree() {
  auto status = tree_.tickOnce();
  StdCoutLogger logger_cout(tree_);

  std::cout << "--- status: " << toStr(status) << "\n\n";

  while (status == NodeStatus::RUNNING && rclcpp::ok()) {

    tree_.sleep(std::chrono::milliseconds(100));

    std::cout << "--- ticking\n";
    status = tree_.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";
  }
  if (status == NodeStatus::SUCCESS) {
    return true;
  } 
  else {
    return false;
  }
};
