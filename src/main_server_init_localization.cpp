#include "rb1_autonomy/servers/server_init_localization.hpp"


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServerInitLocalization>();
    //node->clearVariables();
    rclcpp::spin( node);
    rclcpp::shutdown();

    return 1;
}