#include "rb1_autonomy/autonomy.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomyRb1>();
  node->registerNodes();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}