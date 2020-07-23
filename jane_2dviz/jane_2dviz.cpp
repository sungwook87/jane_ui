#include <ros/ros.h>
#include "src/2dviz.h"

int main(int argc, char **argv)
{
  ROS_INFO("jane_2dviz, Start~ :)");

  ros::init(argc, argv, "jane_2dviz");
  elvis::twodviz g_twodviz = elvis::twodviz();
  ros::spin();
  return 0;
}


