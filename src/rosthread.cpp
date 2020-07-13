#include "rosthread.h"

RosThread::RosThread(int argc, char** pArgv, const char * topic)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv),
        m_topic(topic)
{/** Constructor for the robot thread **/}

RosThread::~RosThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if

    m_pThread->wait();
}//end destructor

bool RosThread::init()
{
    m_pThread = new QThread();
    this->moveToThread(m_pThread);

    connect(m_pThread, &QThread::started, this, &RosThread::run);
    ros::init(m_Init_argc, m_pInit_argv, "gui_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;

//  subscribe //
    sub_pix_diagnostic = nh.subscribe("/diagnostics",1, &RosThread::gpssatCallback, this);
    sub_estgps = nh.subscribe("/estimated_global_position",1,&RosThread::estgpsCallback, this);
    sub_localpose = nh.subscribe("/mavros/local_position/pose", 1, &RosThread::localposeCallback, this);
    sub_imu = nh.subscribe("/mavros/imu/data",1, &RosThread::imuCallback, this);
    sub_status = nh.subscribe("/mavros/statustext/recv", 1, &RosThread::statusCallback, this);
    sub_battery = nh.subscribe("/mavros/battery", 1, &RosThread::batteryCallback, this);
    sub_state = nh.subscribe("/mavros/state", 1, &RosThread::mavstateCallback, this);
    sub_logdata = nh.subscribe("/muin/log_data", 1, &RosThread::logdataCallback, this);
    sub_globalgps = nh.subscribe("/mavros/global_position/global", 1, &RosThread::gpsglobalCallback, this);
    sub_fixgps = nh.subscribe("/mavros/global_position/raw/fix", 1, &RosThread::gpsfixCallback, this);
    sub_compass = nh.subscribe("/mavros/global_position/compass_hdg", 1, &RosThread::gpscompassCallback, this);
    sub_bodyvel = nh.subscribe("/mavros/local_position/velocity_body", 1, &RosThread::bodyvelCallback, this);
    sub_2dmap = nh.subscribe("/mapimage", 1, &RosThread::mapCallback, this);
    sub_cam = nh.subscribe("/zedm/zed_node/rgb/image_rect_color", 1, &RosThread::camCallback, this);
    pub_img = nh.advertise<sensor_msgs::Image>("/output_video", 1);
// service //
    srv_take_off            = nh.serviceClient<muin_px4::take_off>("take_off");
    srv_landing             = nh.serviceClient<muin_px4::landing>("landing");
    srv_emergency_landing   = nh.serviceClient<muin_px4::emergency_landing>("emergency_landing");
    srv_pause_mission       = nh.serviceClient<muin_px4::pause_mission>("pause_mission");
    srv_record_start        = nh.serviceClient<muin_px4::test_srv>("test_srv");
    srv_record_stop         = nh.serviceClient<muin_px4::test_srv>("test_srv");
    srv_local2gps           = nh.serviceClient<muin_px4::start_trig>("start_trigger");
    srv_next_mission        = nh.serviceClient<muin_px4::next_mission>("next_mission");
    srv_prev_mission        = nh.serviceClient<muin_px4::previous_mission>("previous_mission");
    srv_upload_mission      = nh.serviceClient<muin_px4::ui_mission_request>("send_mission_info");
    srv_nonstop_mission     = nh.serviceClient<muin_px4::automatic_mission_start>("auto_mission");
    srv_sethome             = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    srv_kill                = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    srv_rth                 = nh.serviceClient<muin_px4::return_home>("return_home");

    m_pThread->start();
    return true;
}//set up the thread


void RosThread::run()
{
    ros::Rate loop_rate(100);
    QMutex * pMutex;
    while (ros::ok())
    {
        pMutex = new QMutex();

        geometry_msgs::Twist cmd_msg;
        pMutex->lock();
        cmd_msg.linear.x = m_speed;
        cmd_msg.angular.z = m_angle;
        pMutex->unlock();

        //sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
        delete pMutex;
    }//do ros things.
}



void RosThread::gpssatCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  pix_diagnostic = *msg;
  std::string tmp_s;
  for(int i = 0; i < pix_diagnostic.status.size(); i++)
  {
    tmp_s = pix_diagnostic.status[i].name;
    if(tmp_s == "mavros: GPS") pix_diagnostic_GPS = pix_diagnostic.status[i];
  }

  for(int idx = 0; idx < pix_diagnostic_GPS.values.size(); idx++)
  {
    if(pix_diagnostic_GPS.values[idx].key == "Satellites visible")
    {
      std::string tmp_s = pix_diagnostic_GPS.values[idx].value;
      satecount = std::atoi(tmp_s.c_str());
    }
  }
  pMutex->unlock();
  delete pMutex;
  Q_EMIT gpscount(satecount);
}

void RosThread::statusCallback(const mavros_msgs::StatusText::ConstPtr &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  std::string tmp_s;
  tmp_s = msg->text;
  //ROS_INFO("%s", tmp_s.c_str());
  pMutex->unlock();
  delete pMutex;
  Q_EMIT status(tmp_s);
}

void RosThread::logdataCallback(const muin_px4::log_data::ConstPtr &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  std::string tmp_s;
  tmp_s = msg->log_string;
  //ROS_INFO("%s", tmp_s.c_str());
  pMutex->unlock();
  delete pMutex;
  Q_EMIT logdata(tmp_s);
}

void RosThread::mavstateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  std::string mode;
  bool armed;
  std::string armStr;

  if (msg->armed == 1) (armStr="OK");
  else (armStr="NOT");
  mode = msg->mode;
  pMutex->unlock();
  delete pMutex;
  Q_EMIT state(armStr, mode);
}

void RosThread::estgpsCallback(const sensor_msgs::NavSatFix &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  lon = msg.longitude;
  lat = msg.latitude;
  alt = msg.altitude;
  pMutex->unlock();
  delete pMutex;
  Q_EMIT estGPS(lon, lat, alt);
}

void RosThread::gpsglobalCallback(const sensor_msgs::NavSatFix &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  lon = msg.longitude;
  lat = msg.latitude;
  alt = msg.altitude;
  pMutex->unlock();
  delete pMutex;
  Q_EMIT globalGPS(lon, lat, alt);
}

void RosThread::gpsfixCallback(const sensor_msgs::NavSatFix &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  lon = msg.longitude;
  lat = msg.latitude;
  alt = msg.altitude;
  pMutex->unlock();
  delete pMutex;
  Q_EMIT fixGPS(lon, lat, alt);
}

void RosThread::bodyvelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  xvel = msg->twist.linear.x;
  yvel = msg->twist.linear.y;
  zvel = msg->twist.linear.z;
 // ROS_INFO("%lf", xvel);
  pMutex->unlock();
  delete pMutex;
  Q_EMIT bodyvel(xvel, yvel, zvel);
}

void RosThread::gpscompassCallback(const std_msgs::Float64 &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  comp = msg.data;
 // ROS_INFO("%lf", comp);
  pMutex->unlock();
  delete pMutex;
  Q_EMIT compass(comp);
}

void RosThread::imuCallback(const sensor_msgs::Imu &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  double orientation[4] = {0};
  orientation[0] = msg.orientation.x;
  orientation[1] = msg.orientation.y;
  orientation[2] = msg.orientation.z;
  orientation[3] = msg.orientation.w;
  tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
  tf::Matrix3x3 m(q);
  m.getRPY(rR, rP, rY);
  m_heading = rY*180./M_PI;
  pMutex->unlock();
  delete pMutex;
  Q_EMIT imu(m_heading);
}

void RosThread::localposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  m_xPos = msg->pose.position.x;
  m_yPos = msg->pose.position.y;
  m_zPos = msg->pose.position.z;
  geometry_msgs::Quaternion q = msg->pose.orientation;
  geometry_msgs::Vector3 res;
  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(q.x, q.y, q.z, q.w));
  R_FLU2ENU.getRPY(res.x, res.y, res.z);
  m_tPos = res.z*180 / M_PI;
  pMutex->unlock();
  delete pMutex;
  Q_EMIT localpose(m_xPos, m_yPos, m_zPos, m_tPos);
}

void RosThread::batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  m_batt = msg->percentage*100;
  pMutex->unlock();
  delete pMutex;
  Q_EMIT battery(m_batt);
}

void RosThread::mapCallback(const sensor_msgs::ImageConstPtr &msg)
{
  QMutex *pMutex = new QMutex();
  pMutex->lock();
  cv_bridge::CvImagePtr cv_ptr;
  //cv::Mat image;
  try
  {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     imagemap = cv_ptr->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
  }
  pMutex->unlock();
  delete pMutex;
  Q_EMIT mapimage(imagemap);

}

void RosThread::camCallback(const sensor_msgs::ImageConstPtr &msg)
{
  QMutex *pMutex = new QMutex();
  pMutex->lock();
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat image1;
  try
  {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     //imagecam = cv_ptr->image.clone();
     cv::resize(cv_ptr->image, image1, cv::Size(500,500), 0, 0, cv::INTER_LINEAR);
  }
  catch (cv_bridge::Exception& e)
  {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
  }

  pMutex->unlock();
  delete pMutex;
  Q_EMIT camimage(image1);
}


/*
 * Service Functions
 *
 */

void RosThread::fn_take_off()
{
  muin_px4::take_off srv;
  srv_take_off.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("takeoff");
}

void RosThread::fn_landing()
{
  muin_px4::landing srv;
  srv_landing.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("landing");
}

void RosThread::fn_emergency_landing()
{
  muin_px4::emergency_landing srv;
  srv_emergency_landing.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("e-landing");
}

void RosThread::fn_pause_mission()
{
  muin_px4::pause_mission srv;
  srv_pause_mission.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("pause");
}

void RosThread::fn_next_mission()
{
  muin_px4::next_mission srv;
  srv_next_mission.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("next");
}

void RosThread::fn_prev_mission()
{
  muin_px4::previous_mission srv;
  srv_prev_mission.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("previous");
}

void RosThread::fn_record_start()
{
  muin_px4::record_start srv;
  srv.request.value=1;
  srv_record_start.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("cam_start");
}

void RosThread::fn_record_stop()
{
  muin_px4::record_stop srv;
  srv.request.value=2;
  srv_record_stop.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("cam_stop");
}

void RosThread::fn_upload_mission()
{
  muin_px4::ui_mission_request srv;
  srv_upload_mission.call(srv);
  ROS_INFO("%d",srv.response.complete);
  ROS_INFO("mission upload");
}

void RosThread::fn_local2gps()
{
  muin_px4::start_trig srv;
  srv.request.trigger = true;
  srv_local2gps.call(srv);
  ROS_INFO("%d",srv.response.success);
  ROS_INFO("local2gps");
}

void RosThread::fn_nonstop_mission()
{
  muin_px4::automatic_mission_start srv;
  nonstop_mission = !nonstop_mission;
  srv.request.start_mission = nonstop_mission;
  srv_nonstop_mission.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("nonstop mission");
}

void RosThread::fn_kill()
{
  mavros_msgs::CommandLong srv;
  srv.request.command=400;
  srv.request.param2=21196.0;
  srv_kill.call(srv);
  ROS_INFO("%d",srv.response.success);
  ROS_INFO("kill !!");
}

void RosThread::fn_sethome()
{
  mavros_msgs::CommandHome srv;
  srv.request.current_gps = true;
  srv_sethome.call(srv);
  ROS_INFO("%d",srv.response.success);
  ROS_INFO("set home");
}

void RosThread::fn_returnhome()
{
  muin_px4::return_home srv;
  srv_rth.call(srv);
  ROS_INFO("%d",srv.response.result);
  ROS_INFO("Retrun-to-Home");
}


