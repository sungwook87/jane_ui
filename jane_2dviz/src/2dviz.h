#ifndef TWODVIZ_H
#define TWODVIZ_H

#include <ros/ros.h>
#include <ros/time.h>
#include <cstdlib>
#include <tf/tf.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <curl/curl.h>
#include <curl/easy.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "../GeoCoordiConv/GeoCoordConv.h"

#include <std_msgs/UInt8.h>

#include <sensor_msgs/Joy.h>

// System includes
#include "unistd.h"
#include <iostream>
#include <fstream>
#include <sstream>
// OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define GPSMAP_RESOL 0.08 // 20 size // 0.057 straight
#define GPSIMG_SIZE 800
//#define GPS_DEFAULT_LON 127.362826 127.652319435
//#define GPS_DEFAULT_LAT 36.365959  37.8235559629

#define GPS_DEFAULT_LON 127.650216791
#define GPS_DEFAULT_LAT 37.8231037596
using namespace std;

namespace elvis
{
  class twodviz
  {
    private:
    ros::NodeHandle nh;
    ros::Publisher img_pub;
    double gps_[4];
    double estgps_[4];
    double estvio_[4];

    double rR, rP, rY;
    double lR, lP, lY;

    double rRoll, rPitch, rYaw;
    double lRoll, lPitch, lYaw;


    bool GPSValid = false;
    bool estGPSValid = false;
    bool estvioValid = false;

    bool isLoadMap = false;

    int MapZoomLevel = 18;

    double MeterperPixel = GPSMAP_RESOL * pow(2., 20. - static_cast<float>(MapZoomLevel));

  public:
    twodviz();
    ~twodviz();

    void DrawPosImg();
    void MapUpdate(cv::Point2d current_ConvXY);
    void gps_callback(sensor_msgs::NavSatFix msg);
    void estgps_callback(sensor_msgs::NavSatFix msg);
    void estvio_callback(sensor_msgs::NavSatFix msg);


    void timer_callback(const ros::TimerEvent&);
    void CallBackKeyFunc(int event, int x, int y, int flags, void* userdata);
    void rpy_callback(const sensor_msgs::Imu msg);
    void localrpy_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void generate();


    cv::Point2d m_wayPointConv;
    cv::Point2d m_wayPointWGS84;
    cv::Point2d m_mapConvXY;
    cv::Point2d m_currConvXY;
    cv::Point2d m_estConvXY;
    cv::Point2d m_vioConvXY;
    cv::Point2d force_center;

    cv::Mat m_Gpsimg;
    std::list<cv::Point2d> fFootprintPts;
    std::list<cv::Point2d> festFootprintPts;
    std::list<cv::Point2d> fvioFootprintPts;


    cv::Point2d wgs84;
    cv::Point2d estwgs84;
    cv::Point2d viowgs84;


    cv::Point2d gpsRef;

    cv::Point2d Cvt_WGS842Conv(cv::Point2d GPS_lonlat);
    cv::Point2d Cvt_Map2ConvXY(cv::Point pixel_xy, double img_MeterperPix, cv::Point2d origin_conv);
    cv::Point Cvt_ConvXY2Map(cv::Point2d convXY, double img_MeterperPix, cv::Point2d origin_conv);
    cv::Point2d Cvt_Conv2WGS84(cv::Point2d GPS_convXY);
    cv::Mat GetimgFromURL(std::string strm);

  };
}

#endif

