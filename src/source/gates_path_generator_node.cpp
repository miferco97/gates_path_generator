#include "gates_path_generator.hpp"
#include "ros/ros.h"
#include "ros_utils_lib/ros_utils.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, ros_utils_lib::getNodeName("gate_generator_node"));
  std::cout << "Node starting "<< std::endl;
  GatesPathGenerator gate_generator;
  gate_generator.setUp();
  gate_generator.start();
  ros::Rate r(30);
  while (ros::ok())
  {
    gate_generator.run();
    ros::spinOnce();
    r.sleep();
  }
  gate_generator.stop();  
  return 0;
}
