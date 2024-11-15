#include "rb1_autonomy/servers/server_find_object.hpp"

#include "thread"


ServerFindObject::ServerFindObject() : Node("server_find_object"){

  RCLCPP_INFO(get_logger(), "Node initialized [server_find_object]");

  topic_name_laser_ = "scan";
  data_laser_ = std::make_shared<msgLaserScan>();


  init();
  initParameters();
}

void ServerFindObject::init(){
  
  RCLCPP_INFO(get_logger(), "Service server [server_find_object] initialized");
  // iniciar subscriptores
  rclcpp::QoS qos_sensor_scan(10);
  rclcpp::SubscriptionOptions options_sub_laser;
  qos_sensor_scan.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_sensor_scan.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  options_sub_laser.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  subScan_ = create_subscription<msgLaserScan>(topic_name_laser_,rclcpp::QoS(10), std::bind(&ServerFindObject::callbackScan, this, _1),options_sub_laser);
  //------------------------------------
  srvFindObject_ = create_service<srvFindObject>("server_find_object",std::bind(&ServerFindObject::callbackFindObject,this,_1,_2));

}

void ServerFindObject::callbackScan(const msgLaserScan::SharedPtr msg){

  data_laser_ = msg;
  RCLCPP_DEBUG(rclcpp::get_logger("Laser"), "Laser message received");

}

void ServerFindObject::callbackFindObject(const std::shared_ptr<srvFindObject::Request> request, const std::shared_ptr<srvFindObject::Response> response){

  auto start_time = std::chrono::steady_clock::now();

  clearVariables();
  std::string type_object = request->find_object;

  unsigned int count_legs = 0 ;
  unsigned int count_objects = 0 ;

  RCLCPP_DEBUG(get_logger(),"-------------------------------------------");
  RCLCPP_DEBUG(get_logger(), "Object type for search [%s]", type_object.c_str());

  
  // Comprueba si llego la data del láser 
  if(data_laser_->ranges.empty()){
    RCLCPP_ERROR(get_logger(), "No laser reading received");
    return;
  }

  count_legs = findLegsIndex(data_laser_);

  if(count_legs < 2){
      
    RCLCPP_ERROR(get_logger(), "Found [%d] legs", count_legs);
    response->success = false;
  } 
  else{

    RCLCPP_DEBUG(get_logger(), "Found [%d] legs ", count_legs);
    count_objects = findTypeObject(data_laser_,type_object);

    if(count_objects != 1){
        
      RCLCPP_ERROR(get_logger(), "Found [%d] objects of type [%s]", count_objects, type_object.c_str());
      response->success = false;     
    }
    else{

      RCLCPP_DEBUG(get_logger(), "Found [%d] objects of type [%s]", count_objects, type_object.c_str());

      geometry_msgs::msg::Point pos_object;
      pos_object = calculatePointMiddleObject(data_laser_);
      
      response->object_position = pos_object;
      response->success = true;    
    }
  }


  auto end_time = std::chrono::steady_clock::now();
  auto elapse_time = end_time -start_time;
  auto elapse_time_mili = std::chrono::duration_cast<std::chrono::microseconds>(elapse_time);
  RCLCPP_DEBUG(get_logger(), "Service time %ld", elapse_time_mili.count());
}

int ServerFindObject::findLegsIndex(msgLaserScan::SharedPtr msg){

  RCLCPP_DEBUG(get_logger(), " ");
  RCLCPP_DEBUG(get_logger(), "********* [find legs index] ********");
  RCLCPP_DEBUG(get_logger(), " ");

  int count_legs = 0;
  StateFind state_find;
  state_find = StateFind::INIT;

  std::vector<float> intensities = msg->intensities;  

  RCLCPP_DEBUG(get_logger(), "Size [%ld] ", intensities.size());

  for (auto item = intensities.begin(); item != intensities.end(); item++) {

    switch (state_find) {

    case StateFind::INIT:
      if (*item > limit_intensity_laser_detect_) {
        state_find = StateFind::RISING;
        index_legs_.push_back(item - intensities.begin());
        RCLCPP_DEBUG(get_logger(), "Leg number [%d] -- Index added [%ld] -- Intensity [%.2f]", count_legs, item - intensities.begin(), *item);
      }
      break;

    case StateFind::RISING:
      if (*item > limit_intensity_laser_detect_) {
        index_legs_.push_back(item - intensities.begin());
        RCLCPP_DEBUG(get_logger(), "Leg number [%d] -- Index added [%ld] -- Intensity [%.2f]", count_legs, item - intensities.begin(), *item);
      } 
      else {
        count_legs++;
        RCLCPP_DEBUG(get_logger(), "Found [%d] legs", count_legs);
        state_find = StateFind::FALLING;index_legs_.push_back(-1);
      }
      break;

    case StateFind::FALLING:
      if (*item > limit_intensity_laser_detect_) {
        index_legs_.push_back(item - intensities.begin());
        RCLCPP_DEBUG(get_logger(), "Leg number [%d] -- Index added [%ld] -- Intensity [%.2f]", count_legs, item - intensities.begin(), *item);
        state_find = StateFind::RISING;
      }
      break;
    }
  }
  printVector(index_legs_);
  return count_legs;
}

int ServerFindObject::findTypeObject(msgLaserScan::SharedPtr laser, std::string type_object){
  RCLCPP_DEBUG(get_logger(), " ");
  RCLCPP_DEBUG(get_logger(), "************ [Index of middle point of legs] **********");
  RCLCPP_DEBUG(get_logger(), " ");
  
  unsigned int tmp_prom_index = 0;
  unsigned int tmp_sum_index = 0;
  unsigned int tmp_numb_leg = 0;
  unsigned int tmp_count = 0;

  for( auto index_leg = index_legs_.begin(); index_leg != index_legs_.end(); index_leg++){
    if(*index_leg != -1){
      tmp_sum_index = tmp_sum_index + *index_leg;
      tmp_count++;
    }
    else{
      tmp_prom_index = tmp_sum_index/tmp_count;
      index_legs_middle_point_.push_back(tmp_prom_index);
      RCLCPP_DEBUG(get_logger(), "Average index of leg [%d] average [%d]", tmp_numb_leg, tmp_prom_index);
      tmp_numb_leg++;
      tmp_prom_index = 0;
      tmp_sum_index = 0;
      tmp_count = 0;
    }
  }

  RCLCPP_DEBUG(get_logger(), " ");
  RCLCPP_DEBUG(get_logger(), "************ [Finding object type] ************");
  RCLCPP_DEBUG(get_logger(), " ");

  float angle_increment = laser->angle_increment;
  float angle_base = laser->angle_min;
  int count_object = 0;

  for (auto item_i = index_legs_middle_point_.begin();
         item_i != index_legs_middle_point_.end() - 1; item_i++) {

    for (auto item_j = index_legs_middle_point_.begin() + 1;
           item_j != index_legs_middle_point_.end(); item_j++) {

      if (item_j > item_i) {

        RCLCPP_DEBUG(get_logger(), "Verification of index [%d] with index [%d]", *item_i, *item_j);
        float theta_a_leg = (*item_i * angle_increment + angle_base);
        float theta_b_leg = (*item_j * angle_increment + angle_base);
        float d1 = laser->ranges[*item_i];
        float d2 = laser->ranges[*item_j];
        float theta_final = theta_a_leg - theta_b_leg;

        // hallo la distancia entre las patas
        float distance_between_legs =std::sqrt(std::pow(d1, 2) + std::pow(d2, 2) -2 * d1 * d2 * std::cos(theta_final));
        RCLCPP_DEBUG(get_logger(), "Distance between legs [%.3f]", distance_between_legs);
        // Verifico el objeto encontrado

        if (type_object == "shelf") {

          if (distance_between_legs >limit_min_detection_distance_legs_shelf_ && distance_between_legs < limit_max_detection_distance_legs_shelf_) {

            index_shelf_.push_back(*item_i);
            index_shelf_.push_back(*item_j);
            RCLCPP_DEBUG(get_logger(), "Object type [%s] with index [%d] and index [%d]", type_object.c_str(), *item_i, *item_j);            count_object++;
          }
        }
        if (type_object == "station") {

          if (distance_between_legs >limit_min_detection_distance_legs_charge_station_ &&distance_between_legs <limit_max_detection_distance_legs_charge_station_) {

            index_shelf_.push_back(*item_i);
            index_shelf_.push_back(*item_j);
            RCLCPP_DEBUG(get_logger(), "Object type [%s] with index [%d] and with index [%d]", type_object.c_str(), *item_i, *item_j);            count_object++;
          }
        }
      }          
    }
  }  
  printVector(index_shelf_);
  return count_object;
}

void ServerFindObject::initParameters(){

  // Parameters
  declare_parameter("limit_intensity_laser_detect", 0.0);
  limit_intensity_laser_detect_ = get_parameter("limit_intensity_laser_detect").as_double();
  RCLCPP_INFO(get_logger(), "Laser detection intensity limit [%.3f] ",
              limit_intensity_laser_detect_);
  //----------------------------------------------------------
  declare_parameter("limit_min_detection_distance_legs_shelf", 0.55);
  limit_min_detection_distance_legs_shelf_ = get_parameter("limit_min_detection_distance_legs_shelf").as_double();
  RCLCPP_INFO(get_logger(),
              "Minimum detection distance for shelf legs [%.3f] ",
              limit_min_detection_distance_legs_shelf_);
  //----------------------------------------------------------
  this->declare_parameter("limit_max_detection_distance_legs_shelf", 0.70);
  limit_max_detection_distance_legs_shelf_ = get_parameter("limit_max_detection_distance_legs_shelf").as_double();
  RCLCPP_INFO(get_logger(),
              "Maximum detection distance for shelf legs [%.3f] ",
              limit_max_detection_distance_legs_shelf_);
  //----------------------------------------------------------
  declare_parameter("limit_min_detection_distance_legs_charge_station", 0.25);
  limit_min_detection_distance_legs_charge_station_ = get_parameter("limit_min_detection_distance_legs_charge_station").as_double();
  RCLCPP_INFO(get_logger(),
              "Minimum detection distance for charge station legs [%.3f] ",
              limit_min_detection_distance_legs_charge_station_);
  //----------------------------------------------------------
  declare_parameter("limit_max_detection_distance_legs_charge_station", 0.35);
  limit_max_detection_distance_legs_charge_station_ = get_parameter("limit_max_detection_distance_legs_charge_station").as_double();
  RCLCPP_INFO(get_logger(),
              "Maximum detection distance for charge station legs [%.3f] ",
              limit_max_detection_distance_legs_charge_station_);
  //----------------------------------------------------
  declare_parameter("filter_laser_distance_noise", 0.35);
  filter_laser_distance_noise_ = get_parameter("filter_laser_distance_noise").as_double();
  RCLCPP_INFO(get_logger(), "Laser distance noise filter [%.3f] ",
              filter_laser_distance_noise_);


}

void ServerFindObject::printVector(const std::vector<int>& my_vector){

  std::stringstream ss;
  ss << "[";
  for( auto item=my_vector.begin(); item !=my_vector.end(); item++){
    ss << *item;  
    if( item != my_vector.end()-1) ss<< ",";
  }
  ss << "]";
  RCLCPP_DEBUG_STREAM(get_logger(), "Vector content: " << ss.str());
}

void ServerFindObject::clearVariables(){

  index_legs_.clear();
  index_legs_middle_point_.clear();
  index_shelf_.clear();
}

msgPoint ServerFindObject::calculatePointMiddleObject(msgLaserScan::SharedPtr laser){

  RCLCPP_DEBUG(get_logger(), " ");
  RCLCPP_DEBUG(this->get_logger(), "**************Calculate middle point *************");  RCLCPP_DEBUG(get_logger(), " ");

  float angle_increment = laser->angle_increment;
  float angle_base = laser->angle_min;

  RCLCPP_DEBUG(this->get_logger(), "angle_base  [%.3f]", angle_base);
  RCLCPP_DEBUG(this->get_logger(), "angle_increment  [%.3f]",
                 angle_increment);

  int index_leg_1 = index_shelf_[0];
  int index_leg_2 = index_shelf_[1];

  float theta_a_leg = (index_leg_1 * angle_increment + angle_base);
  float theta_b_leg = (index_leg_2 * angle_increment + angle_base);
  RCLCPP_DEBUG(this->get_logger(), "theta_a_leg [%.3f]", theta_a_leg);
  RCLCPP_DEBUG(this->get_logger(), "theta_b_leg [%.3f]", theta_b_leg);
  float d1 = laser->ranges[index_leg_1];
  float d2 = laser->ranges[index_leg_2];
  RCLCPP_DEBUG(this->get_logger(), "d1 [%.3f]", d1);
  RCLCPP_DEBUG(this->get_logger(), "d2 [%.3f]", d2);
  float dt_alfa = d1 * std::cos(theta_a_leg) + d2 * std::cos(theta_b_leg);
  float dt_beta = d1 * std::sin(theta_a_leg) + d2 * std::sin(theta_b_leg);
  RCLCPP_DEBUG(this->get_logger(), "dt_alfa [%.3f]", dt_alfa);
  RCLCPP_DEBUG(this->get_logger(), "dt_beta [%.3f]", dt_beta);
  float dt = std::sqrt(std::pow(dt_alfa, 2) + std::pow(dt_beta, 2)) / 2;
  float theta_total = std::atan2(dt_beta, dt_alfa);
  RCLCPP_DEBUG(this->get_logger(), "dt [%.3f]", dt);
  RCLCPP_DEBUG(this->get_logger(), "theta_total [%.3f]", theta_total);
  float c = std::sqrt(std::pow(d2, 2) + std::pow(dt, 2) -
                        2 * d2 * dt * std::cos(abs(theta_total - theta_b_leg)));
  RCLCPP_DEBUG(this->get_logger(), "c [%.3f]", c);
  float alfa_theta = abs(std::acos(
        (std::pow(c, 2) + std::pow(dt, 2) - std::pow(d2, 2)) / (2 * c * dt)));
  RCLCPP_DEBUG(this->get_logger(), "alfa_theta [%.3f]", alfa_theta);
  float theta_final = PI / 2 + theta_total - alfa_theta;
  RCLCPP_DEBUG(this->get_logger(), "theta_final [%.3f]", theta_final);

  geometry_msgs::msg::Point middle_point_object;
  middle_point_object.x = dt;
  middle_point_object.y = theta_total;
  middle_point_object.z = theta_final;

  return middle_point_object;
}