#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "rb1_interfaces/srv/find_object.hpp"

#include "vector"
#include "iostream"
#include "functional"
#include "chrono"

using namespace std::chrono_literals;
using namespace std::placeholders;

using msgLaserScan = sensor_msgs::msg::LaserScan;
using msgPoint = geometry_msgs::msg::Point;

using srvFindObject = rb1_interfaces::srv::FindObject;


#define PI 3.141592653589793238462643
enum class StateFind { INIT, FALLING, RISING };
  
class ServerFindObject : public rclcpp::Node {

public:

    ServerFindObject();

    void callbackScan(const msgLaserScan::SharedPtr);
    void callbackFindObject(const std::shared_ptr<srvFindObject::Request> , const std::shared_ptr<srvFindObject::Response>);


private:



    rclcpp::Subscription<msgLaserScan>::SharedPtr subScan_;
    rclcpp::Service<srvFindObject>::SharedPtr srvFindObject_;
    
    std::vector<int> index_legs_;
    std::vector<int> index_shelf_;
    std::vector<unsigned int> index_legs_middle_point_;

    std::string topic_name_laser_;
    msgLaserScan::SharedPtr data_laser_;

    void init();
    void initParameters();
    void clearVariables();

    int findLegsIndex(msgLaserScan::SharedPtr);
    int findTypeObject(msgLaserScan::SharedPtr,std::string);
    msgPoint calculatePointMiddleObject(msgLaserScan::SharedPtr);
    
    // print functions
    void printVector(const std::vector<int>& );

    // params
    double limit_intensity_laser_detect_;
    double limit_min_detection_distance_legs_shelf_;
    double limit_max_detection_distance_legs_shelf_;
    double limit_min_detection_distance_legs_charge_station_;
    double limit_max_detection_distance_legs_charge_station_;
    double filter_laser_distance_noise_;

};