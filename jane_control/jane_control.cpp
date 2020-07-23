#include <ros/ros.h>
#include "./src/fncs_header.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "jane_ctrl");
    ROS_INFO("jane_ctrl, Start~ :)");
    jane_control_class * jane_controller = new jane_control_class();

    jane_controller->nh.getParam("wp_vel",jane_controller->wp_vel);
    jane_controller->nh.getParam("height_vel",jane_controller->height_vel);
    jane_controller->nh.getParam("landing_vel",jane_controller->landing_vel);
    jane_controller->nh.getParam("yaw_vel",jane_controller->yaw_vel);//rad
    jane_controller->nh.getParam("take_off_height",jane_controller->take_off_height);
    jane_controller->nh.getParam("threshold_distance",jane_controller->threshold_distance);
    jane_controller->nh.getParam("arrive_distance",jane_controller->arrive_distance);
    jane_controller->nh.getParam("threshold_yaw",jane_controller->threshold_yaw);
    jane_controller->nh.getParam("LOS_radius",jane_controller->LOS_radius);
    jane_controller->nh.getParam("auto_arrive_delay",jane_controller->auto_arrive_delay);
    jane_controller->nh.getParam("test_mode",jane_controller->test_mode);


    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
//        ROS_INFO("uav_control UAV control mode : %d",jane_controller->UAV_control_mode);
        jane_controller->spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
