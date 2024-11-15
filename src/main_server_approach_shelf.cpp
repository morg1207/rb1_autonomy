#include "rb1_autonomy/servers/server_approach_shelf.hpp"


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServerApproachShelf>();
    rclcpp::spin( node);
    rclcpp::shutdown();

    return 1;
}