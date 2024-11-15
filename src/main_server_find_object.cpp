#include "rb1_autonomy/servers/server_find_object.hpp"


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServerFindObject>();
    rclcpp::spin( node);
    rclcpp::shutdown();

    return 1;
}