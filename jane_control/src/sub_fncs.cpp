#include "fncs_header.hpp"



void jane_control_class::cb_pix_state(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

    armed_flag = current_state.armed;

    int current_system_status = current_state.system_status;
    if(current_system_status == 3) UAV_control_mode = standby_mode;
    else if (current_system_status < 3 || current_system_status > 5) UAV_control_mode = not_ready_mode;

    bool current_guide = current_state.guided;
    if(!current_guide){
        // log
        jane_ui::log_data log_data;
        log_data.string_color = LOG_EMERGENCY;
        log_data.log_string = "Not guided, please check localization";
        pub_log_data.publish(log_data);
    }
}

void jane_control_class::cb_pix_local(const nav_msgs::Odometry::ConstPtr &msg)
{
    pix_poseLocal = *msg;
    current_orientation = pix_poseLocal.pose.pose.orientation;
}

void jane_control_class::cb_pix_home(const mavros_msgs::HomePosition::ConstPtr &msg)
{
    pix_home = *msg;
}
