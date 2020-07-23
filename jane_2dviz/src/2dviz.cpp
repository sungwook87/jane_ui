#include "2dviz.h"

using namespace elvis;
using namespace std;
using namespace cv;

twodviz::twodviz()
{
  ros::NodeHandle nh;
  static ros::Subscriber sub_gps = nh.subscribe("/mavros/global_position/global", 1, &twodviz::gps_callback, this);
  static ros::Subscriber sub_rpy = nh.subscribe("/mavros/imu/data", 1, &twodviz::rpy_callback, this);
  static ros::Subscriber sub_estgps = nh.subscribe("/estimated_global_position_fixed", 1, &twodviz::estgps_callback, this);
  static ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), &twodviz::timer_callback,this);
  static ros::Subscriber sub_localrpy = nh.subscribe("/mavros/local_position/pose", 1, &twodviz::localrpy_callback, this);

  img_pub = nh.advertise<sensor_msgs::Image>("/mapimage",1000);

  cv::namedWindow("GPS", 1);

  wgs84 = cv::Point2d (GPS_DEFAULT_LON, GPS_DEFAULT_LAT);
  gpsRef = cv::Point2d (24,0);

  std::ifstream ref("../../../src/samsung_gtc/gpsref");
  string temp;
  ref >> temp >> gpsRef.x;
  ref >> temp >> gpsRef.y;
  ref.close();

  m_mapConvXY = twodviz::Cvt_WGS842Conv(wgs84);
  MapUpdate(m_mapConvXY);
}

twodviz::~twodviz()
{

}

cv::Point2d twodviz::Cvt_Map2ConvXY(cv::Point pixel_xy, double img_MeterperPix, cv::Point2d origin_conv)
{
    cv::Point2d convXY;

    convXY.x = (double)(((double)pixel_xy.x - GPSIMG_SIZE / 2) * img_MeterperPix + origin_conv.x);
    convXY.y = (double)((-(double)pixel_xy.y + GPSIMG_SIZE / 2) * img_MeterperPix + origin_conv.y);

    return convXY;
}

cv::Point twodviz::Cvt_ConvXY2Map(cv::Point2d convXY, double img_MeterperPix, cv::Point2d origin_conv)
{
    cv::Point pixel_xy;

    pixel_xy.x = (int)((double)(convXY.x - origin_conv.x) / img_MeterperPix + GPSIMG_SIZE / 2);
    pixel_xy.y = (int)((double)(convXY.y - origin_conv.y) / img_MeterperPix - GPSIMG_SIZE / 2);

    pixel_xy.y *= -1;
    return pixel_xy;
}

cv::Point2d twodviz::Cvt_Conv2WGS84(cv::Point2d GPS_convXY)
{
    CGeoCoordConv CoordConv;
    CoordConv.SetSrcType(kBessel1984, kTmMid);
    CoordConv.SetDstType(kWgs84, kGeographic);

    double lat, lon;
    CoordConv.Conv(GPS_convXY.x, GPS_convXY.y, lon, lat);
    cv::Point2d result_lonlat = cv::Point2d(lon, lat);
    return result_lonlat;
}

cv::Point2d twodviz::Cvt_WGS842Conv(cv::Point2d GPS_lonlat)
{
    CGeoCoordConv CoordConv;
    CoordConv.SetSrcType(kWgs84, kGeographic);
    CoordConv.SetDstType(kBessel1984, kTmMid);

    double convX, convY;
    CoordConv.Conv(GPS_lonlat.x, GPS_lonlat.y, convX, convY);
    cv::Point2d result_convXY = cv::Point2d(convX, convY);
    return result_convXY;
}

size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    std::ostringstream *stream = (std::ostringstream*)userdata;
    size_t count = size * nmemb;
    stream->write(ptr, count);
    return count;
}

cv::Mat twodviz::GetimgFromURL(std::string strm)
{
    CURL *curl;
    CURLcode res;
    std::ostringstream stream;
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, strm.c_str()); //the img url
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data); // pass the writefunction
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream); // pass the stream ptr when the writefunction is called
    res = curl_easy_perform(curl); // start curl
    std::string output = stream.str(); // convert the stream into a string
    curl_easy_cleanup(curl); // cleanup
    std::vector<char> data = std::vector<char>(output.begin(), output.end()); //convert string into a vector
    cv::Mat data_mat = cv::Mat(data); // create the cv::Mat datatype from the vector
    cv::Mat image = cv::imdecode(data_mat, 1); //read an image from memory buffer

    return image;
}

void twodviz::MapUpdate(cv::Point2d current_ConvXY)
{
    std::stringstream strm;
    // google map
    // strm << "http://maps.googleapis.com/maps/api/staticmap?center=";
    // naver map
//    strm << "https://naveropenapi.apigw.ntruss.com/map-static/v2/raster?w=640&h=640"
//            "&center=127.362826,36.365959"
//            "&level=16"
//            "&X-NCP-APIGW-API-KEY-ID=8dep6ty1oi"
//            "&X-NCP-APIGW-API-KEY=56UrK9Y7qnfzotm91RXamcZy6SSZsM2SkHzygXV4";

    strm << "https://naveropenapi.apigw.ntruss.com/map-static/v2/raster?w=800&h=800";
    cv::Point2d wgs84XY = Cvt_Conv2WGS84(current_ConvXY);
    strm <<"&center="<<wgs84XY.x<<","<<wgs84XY.y;
    strm <<"&level=18";
    strm <<"&X-NCP-APIGW-API-KEY-ID=8dep6ty1oi";
    strm <<"&X-NCP-APIGW-API-KEY=56UrK9Y7qnfzotm91RXamcZy6SSZsM2SkHzygXV4";
    strm << "&maptype=satellite";
//    strm.precision(8);
//    strm << wgs84XY.y << ",";
//    strm.precision(9);
//    strm << wgs84XY.x;
//    strm << "&zoom=";
//    strm << MapZoomLevel;
//    strm << "&size=640x640";
//    strm << "&maptype=satellite";
//    strm << "&sensor=false";

    m_Gpsimg = GetimgFromURL(strm.str());

    isLoadMap = true;
    m_mapConvXY = current_ConvXY;
}

void twodviz::DrawPosImg()
{
    cv::Mat _Map = m_Gpsimg.clone();
    int face[] = { cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX, cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                   cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC };

    // old path drawing
    std::list<cv::Point2d>::iterator i = fFootprintPts.begin();
    for (; i != fFootprintPts.end(); i++)
        cv::circle(_Map, Cvt_ConvXY2Map(*i, MeterperPixel, m_mapConvXY), 1, cvScalar(0, 0, 255), 1);
    std::list<cv::Point2d>::iterator j = festFootprintPts.begin();
    for (; j != festFootprintPts.end(); j++)
        cv::circle(_Map, Cvt_ConvXY2Map(*j, MeterperPixel, m_mapConvXY), 1, cvScalar(255, 0, 0), 1);

    // Crt Position
    cv::Point robotPoint = Cvt_ConvXY2Map(m_currConvXY + gpsRef, MeterperPixel, m_mapConvXY);
    cv::Point estrobotPoint = Cvt_ConvXY2Map(m_estConvXY + gpsRef, MeterperPixel, m_mapConvXY);

    cv::circle(_Map, robotPoint, 5, cvScalar(0, 0, 255), 2);
    cv::circle(_Map, estrobotPoint, 5, cvScalar(255, 0, 0), 2);

    int dx = (int)(cos(rY)*15);
    int dy = -(int)(sin(rY)*15);
    int ldx = (int)(cos(lY)*15);
    int ldy = -(int)(sin(lY)*15);
    cv::line(_Map, robotPoint + cv::Point(dx, dy), robotPoint, cvScalar(0, 255, 0), 2);
    cv::line(_Map, estrobotPoint + cv::Point(ldx, ldy), estrobotPoint, cvScalar(0, 0, 255), 2);

    //cv::imshow("GPS", _Map);

    //cvWaitKey(1);


    cv_bridge::CvImage bridge;
    _Map.copyTo(bridge.image);
    bridge.header.frame_id = "frameID";
    bridge.header.stamp = ros::Time::now();
    if(_Map.type() == CV_8UC1)
    {
        bridge.encoding = sensor_msgs::image_encodings::MONO8;
    }
    else if(_Map.type() == CV_8UC3)
    {
        bridge.encoding = sensor_msgs::image_encodings::BGR8;
    }
    else if(_Map.type() == CV_32FC1)
    {
        bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    }
    else if(_Map.type() == CV_16UC1)
    {
        bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    }
    else
    {
        std::cout <<"Error : mat type" << std::endl;
    }
    img_pub.publish(bridge);   // --> publish MAP
}

void twodviz::gps_callback(sensor_msgs::NavSatFix msg)
{
    gps_[0] = msg.longitude;
    gps_[1] = msg.latitude;
    gps_[2] = msg.altitude;

   /* cout << "Lon:" << gps_[0] << ", "
         << "Lat:" << gps_[1] << ", "
         << "Alt:" << gps_[2] << endl;
*/
    wgs84.x = static_cast<double>(gps_[0]);
    wgs84.y = static_cast<double>(gps_[1]);
    GPSValid = true;
}

void twodviz::estgps_callback(sensor_msgs::NavSatFix msg)
{
    estgps_[0] = msg.longitude;
    estgps_[1] = msg.latitude;
    estgps_[2] = msg.altitude;

//    cout << "estLon:" << estgps_[0] << ", "
//         << "estLat:" << estgps_[1] << ", "
//         << "estAlt:" << estgps_[2] << endl;
    estwgs84.x = static_cast<double>(estgps_[0]);
    estwgs84.y = static_cast<double>(estgps_[1]);
    estGPSValid = true;
}

void twodviz::rpy_callback(const sensor_msgs::Imu msg)
{
    double orientation[4] = {0};
    orientation[0] = msg.orientation.x;
    orientation[1] = msg.orientation.y;
    orientation[2] = msg.orientation.z;
    orientation[3] = msg.orientation.w;

    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    m.getRPY(rR, rP, rY);
    rRoll = rR * 180./CV_PI;    rPitch = rP*180./CV_PI;    rYaw = rY*180./CV_PI;
    //cout << gpsRef.x << gpsRef.y << endl;
   // ROS_INFO("imu yaw: %.5f", rYaw);
}

void twodviz::localrpy_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double orientation[4] = {0};
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;

    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    m.getRPY(lR, lP, lY);
    lRoll = lR * 180./CV_PI;    lPitch = lP*180./CV_PI;    lYaw = lY*180./CV_PI;
    //cout << gpsRef.x << gpsRef.y << endl;
    //ROS_INFO("local yaw: %.5f", lYaw);
}

void twodviz::timer_callback(const ros::TimerEvent &)
{
  m_currConvXY = Cvt_WGS842Conv(wgs84);
  m_estConvXY = Cvt_WGS842Conv(estwgs84);

  fFootprintPts.push_back(m_currConvXY + gpsRef);
  festFootprintPts.push_back(m_estConvXY + gpsRef);

  if (fFootprintPts.size() > 10000)
      fFootprintPts.pop_front();
  if (fFootprintPts.size() > 10000)
      festFootprintPts.pop_front();

  // Map Update
  int DistanceDiff = (int)sqrt(pow((m_mapConvXY.x -m_currConvXY.x), 2) + pow((m_mapConvXY.y - m_currConvXY.y), 2));
  if (isLoadMap && DistanceDiff >= GPSIMG_SIZE / 2 * MeterperPixel)
      MapUpdate(m_currConvXY);
  if (isLoadMap) DrawPosImg();
}

