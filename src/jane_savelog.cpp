#include <ros/ros.h>
#include <ros/time.h>
#include <cstdlib>
#include <tf/tf.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "unistd.h"
#include <iostream>
#include <fstream>
#include <sstream>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Altitude.h>

using namespace std;


class Savelog
{
public:
  ros::NodeHandle nh;
  ros::Subscriber sub_gps;
  ros::Subscriber sub_alt;
  ros::Subscriber sub_local;

  double pose_[3];
  double amsl;
  struct tm *nowTime;

  FILE *logFile;

  Savelog(string path){
    sub_gps = nh.subscribe("/mavros/global_position/global", 1, &Savelog::gps_callback, this);
    sub_alt = nh.subscribe("/mavros/altitude", 1, &Savelog::alt_callback, this);
    sub_local = nh.subscribe("/mavros/local_position/pose", 1, &Savelog::local_callback, this);


    logFile = NULL;

    char logfileName[200] = {0};
    time_t timer;
    time(&timer);
    nowTime = localtime(&timer);

    sprintf(logfileName, "%s/KAIST_log_%04d%02d%02d_%02d%02d%02d.txt", path.c_str(), nowTime->tm_year + 1900, nowTime->tm_mon + 1, nowTime->tm_mday, nowTime->tm_hour, nowTime->tm_min, nowTime->tm_sec);
    //sprintf(logfileName, "%s.txt", path.c_str());

    logFile = fopen(logfileName, "wt");

    if(logFile != NULL){
                        fprintf(logFile, "time\tevent\tlon\tlat\talt\tpose_x\tpose_y\tpose_z\n");

      ROS_INFO("log file open success.");
    }else
      ROS_ERROR("log file open failed.");
  }

  ~Savelog(){
    if(logFile != NULL)
      fclose(logFile);
  }

  void gps_callback(sensor_msgs::NavSatFix msg)
  {
    double gps_[2];
    gps_[0] = msg.longitude;
    gps_[1] = msg.latitude;

    if(logFile != NULL){
      fprintf(logFile, "%04d%02d%02d_%02d%02d%02d\t 1\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\n", nowTime->tm_year + 1900, nowTime->tm_mon + 1,
              nowTime->tm_mday, nowTime->tm_hour, nowTime->tm_min, nowTime->tm_sec, gps_[0], gps_[1], amsl, pose_[0], pose_[1], pose_[2]);
    }

  }

  void alt_callback(mavros_msgs::Altitude msg)
  {
    amsl = msg.amsl;
  }

  void local_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    pose_[0] = msg->pose.orientation.x;
    pose_[1] = msg->pose.orientation.y;
    pose_[2] = msg->pose.orientation.z;
  }

};

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "savelog");

    if(argc != 2){
      ROS_ERROR("usage: saveStar /home/nvidia/logs");
      return -1;
    }

    Savelog savelog(argv[1]);

    ros::spin();

    return 0;
  }
