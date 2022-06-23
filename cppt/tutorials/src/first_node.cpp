/**
 * @file first_node.cpp
 * @author Loren Lyttle (loren.lyttle@vecnarobotics.com)
 * @brief
 * @version 0.1
 * @date 2022-06-10
 *
 * @copyright Copyright (c) 2022
 *
 * RUN THIS LINE TO COMPILE CODE
 * g++ first_node.cpp -o first_node -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime
 *
 * WEBSITE DOCUMENTATION
 * https://lsi.vc.ehu.eus/pablogn/investig/ROS/ROScpp.pdf
*/

#include <ros/ros.h>
#include "std_msgs/String.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"first_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);

  while(ros::ok())
  {
    ROS_INFO("Help me Obi-Wan Kenobi, you'are my only hope");
  }

  return 0;
}