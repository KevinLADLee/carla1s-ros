
#include "carla_maps.h"

CarlaMaps::CarlaMaps() {

    carla_world_info_sub_ = nh_.subscribe<std_msgs::String>("/carla/map", 1, boost::bind(&CarlaMaps::CarlaMapCallback, this, _1));

}

void CarlaMaps::CarlaMapCallback(const std_msgs::StringConstPtr &carla_map_msg) {
//  std::copy(carla_map_msg->data.begin(), carla_map_msg->data.end(),opendrive_xml_.begin());
  opendrive_xml_ = carla_map_msg->data;
  pugi_result = pugi_doc_.load_string(opendrive_xml_.c_str());
  for (pugi::xml_node node_road : pugi_doc_.child("OpenDRIVE").children("road")){
    for(auto node_object : node_road.child("objects").children("object")){
      auto type = std::string(node_object.attribute("type").as_string());
      if (type == "parking"){
        std::cout << "ParkingSpot" << std::endl;
        std::cout << node_object.attribute("s").as_double() << std::endl;
        std::cout << node_object.attribute("t").as_double() << std::endl;
        std::cout << node_object.attribute("hdg").as_double() << std::endl;
        std::cout << node_object.attribute("width").as_double() << std::endl;
        std::cout << node_object.attribute("length").as_double() << std::endl;
      }
    }
  }

}
