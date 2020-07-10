#ifndef ___ROSTHREAD_H___
#define ___ROSTHREAD_H___

#include <QtCore>
#include <QThread>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>
#include <iostream>
#include "assert.h"
#include <tf/tf.h>

// msg
#include <ros/ros.h>
#include <ros/network.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/StatusText.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Service
#include "muin_px4/pause_mission.h"
#include "muin_px4/landing.h"
#include "muin_px4/take_off.h"
#include "muin_px4/record_start.h"
#include "muin_px4/record_stop.h"
#include "muin_px4/send_mission_info.h"
#include "muin_px4/next_mission.h"
#include "muin_px4/previous_mission.h"
#include "muin_px4/emergency_landing.h"
#include "muin_px4/test_srv.h"
#include "muin_px4/start_trig.h"
#include "muin_px4/log_data.h"
#include "muin_px4/ui_mission_request.h"
#include "muin_px4/automatic_mission_start.h"

class RosThread : public QObject {
  Q_OBJECT
private:
    int m_Init_argc;
    char** m_pInit_argv;
    const char * m_topic;

    diagnostic_msgs::DiagnosticStatus pix_diagnostic_GPS;
    diagnostic_msgs::DiagnosticArray pix_diagnostic;

    unsigned int satecount;


    double m_speed;
    double m_angle;
    double m_batt;

    double m_xPos;
    double m_yPos;
    double m_zPos;
    double m_tPos;
    double lon, lat, alt;
    double m_heading;
    double rR, rP, rY;
    double xvel, yvel, zvel;
    double comp;
    cv::Mat imagemap;
    cv::Mat imagecam;

   // cv::Mat image1;


   // double lon, lat, alt;

    double m_maxRange;
    double m_minRange;

    QThread * m_pThread;

    ros::Publisher pub_img;
    ros::Subscriber pose_listener;
    ros::Subscriber sub_pix_diagnostic;
    ros::Subscriber sub_estgps;
    ros::Subscriber sub_localpose;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_status;
    ros::Subscriber sub_battery;
    ros::Subscriber sub_state;
    ros::Subscriber sub_logdata;

    ros::Subscriber sub_globalgps;
    ros::Subscriber sub_fixgps;
    ros::Subscriber sub_compass;
    ros::Subscriber sub_bodyvel;

    ros::Subscriber sub_2dmap;
    ros::Subscriber sub_cam;


    ros::ServiceClient srv_take_off;
    ros::ServiceClient srv_landing;
    ros::ServiceClient srv_pause_mission;
    ros::ServiceClient srv_emergency_landing;
    ros::ServiceClient srv_record_start;
    ros::ServiceClient srv_record_stop;
    ros::ServiceClient srv_upload_mission;
    ros::ServiceClient srv_next_mission;
    ros::ServiceClient srv_prev_mission;
    ros::ServiceClient srv_local2gps;
    ros::ServiceClient srv_nonstop_mission;



public:
    RosThread(int argc, char **pArgv, const char * topic  = "/odom");
    virtual ~RosThread();

    bool init();

    void estgpsCallback(const sensor_msgs::NavSatFix & msg);
    void gpssatCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr &msg);
    void localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu &msg);
    void statusCallback(const mavros_msgs::StatusText::ConstPtr &msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg);
    void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
    void logdataCallback(const muin_px4::log_data::ConstPtr &msg);
    void gpsglobalCallback(const sensor_msgs::NavSatFix &msg);
    void gpsfixCallback(const sensor_msgs::NavSatFix &msg);
    void gpscompassCallback(const std_msgs::Float64 &msg);
    void bodyvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void mapCallback(const sensor_msgs::ImageConstPtr &msg);
    void camCallback(const sensor_msgs::ImageConstPtr &msg);

    void fn_take_off();
    void fn_landing();
    void fn_emergency_landing();
    void fn_pause_mission();
    void fn_record_start();
    void fn_record_stop();
    void fn_upload_mission();
    void fn_next_mission();
    void fn_prev_mission();
    void fn_local2gps();
    void fn_nonstop_mission();

    Q_SLOT void run();

    Q_SIGNAL void gpscount(unsigned int);
    Q_SIGNAL void estGPS(double, double, double);
    Q_SIGNAL void fixGPS(double, double, double);
    Q_SIGNAL void bodyvel(double, double, double);
    Q_SIGNAL void compass(double);
    Q_SIGNAL void localpose(double, double, double, double);
    Q_SIGNAL void imu(double);
    Q_SIGNAL void status(std::string);
    Q_SIGNAL void logdata(std::string);
    Q_SIGNAL void battery(double);
    Q_SIGNAL void state(std::string, std::string);
    Q_SIGNAL void globalGPS(double, double, double);
    Q_SIGNAL void mapimage(cv::Mat);
    Q_SIGNAL void camimage(cv::Mat);


};
#endif
