#include "fncs_header.hpp"

//UAV service server
bool jane_control_class::fn_take_off(jane_ui::take_off::Request &req,
jane_ui::take_off::Response &res){
    log_cnt = true;

    if(armed_flag == false){
        offboard_flag = true;
        if(uav_wp_local.size() > 0 && UAV_control_mode == standby_mode)
        {
            target_position.type_mask = position_control_type;
            target_position.position.x = 0;
            target_position.position.y = 0;
            target_position.position.z = take_off_height;
            target_position.yaw = Quat2Angle(current_orientation).z;
            setpoint_raw_pub.publish(target_position);
            UAV_control_mode = take_off_mode;

            // log
            loglog = "TakeOff wait";
            log_pub_ros_info(LOG_IMPORTANT,loglog);

            res.result = true;
            return true;
        }
        else{
            UAV_control_mode = not_ready_mode;

            // log
            loglog = "Cannot takeoff, check waypoint & UAV state";
            log_pub_ros_info(LOG_IMPORTANT,loglog);

            res.result =false;
            return false;
        }
    }
    else{
        // log
        loglog = "Maybe already finished TakeOff";
        log_pub_ros_info(LOG_IMPORTANT,loglog);

        res.result =false;
        return false;
    }
}

bool jane_control_class::fn_landing(jane_ui::landing::Request &req,
jane_ui::landing::Response &res){
    log_cnt = true;

    UAV_control_mode = landing_mode;
    res.result = true;
}

bool jane_control_class::fn_send_mission_info(jane_ui::ui_mission_request::Request &req,
jane_ui::ui_mission_request::Response &res){
    log_cnt = true;

    if(current_state.system_status == 3 && armed_flag == false){
        jane_ui::linenumin_waypointout data;
        data.request.line_num = 100;
        client_request_waypoint.call(data);
        wp_size = data.response.total_line + 1;
        uav_wp_local.clear();
        uav_heading.clear();
        res.complete = true;
        if(wp_size>0){
            uav_wp_local.resize(wp_size);
            uav_heading.resize(wp_size);
            bool take_wp;
            for (int line = 0;line < wp_size;line++) {
                data.request.line_num = line;
                client_request_waypoint.call(data);
                take_wp = data.response.exist;
                if (take_wp){
                    uav_wp_local[line] = data.response.waypoint.position;
                    uav_heading[line] = data.response.waypoint.orientation.w;
                    if(line == wp_size-1){
                        // log
                        loglog = "There is " + std::to_string(wp_size) + " mission";
                        log_pub_ros_info(LOG_IMPORTANT,loglog);
                        log_cnt = true;

                        UAV_control_mode = standby_mode;
                    }
                }
                else{
                    line = wp_size;
                    // log
                    loglog = "Something wrong with waypoint request!";
                    log_pub_ros_info(LOG_ERROR,loglog);
                    log_cnt = true;

                    res.complete = false;
                }
            }
        }
        else {
            loglog = "There is no mission, please check mission";
            log_pub_ros_info(LOG_ERROR,loglog);
            log_cnt = true;

            res.complete = false;
        }
    }
    else{
        // log
        loglog = "Not a proper state for upload, please not insert mission";
        log_pub_ros_info(LOG_ERROR,loglog);
        log_cnt = true;

        res.complete =false;
    }
    return res.complete;
}

bool jane_control_class::fn_automatic_mission_start(jane_ui::automatic_mission_start::Request &req,
           jane_ui::automatic_mission_start::Response &res) {
    log_cnt = true;

    auto_flag = req.start_mission;
    ROS_INFO("auto_flag : %d",auto_flag);
    std::string auto_auto = auto_flag? "true" : "false";
    loglog = "auto_flag : " + auto_auto;
    log_pub_ros_info(LOG_NORMAL, loglog);
    res.result = true;
}

bool jane_control_class::fn_next_mission(jane_ui::next_mission::Request &req,
jane_ui::next_mission::Response &res){
    log_cnt = true;

    wp_idx++;
    if (wp_idx<wp_size){
        UAV_control_mode = move_wp_mode;
//        // log
//        loglog = "Depart to next waypoint";
//        log_pub_ros_info(LOG_NORMAL,loglog);
//        log_cnt = true;

        res.result = true;
    }
    else {
        wp_idx--;
        UAV_control_mode = hover_wp_mode;
        // log
        loglog = "This is the last mission, cannot go to next waypoint";
        log_pub_ros_info(LOG_ERROR,loglog);
        log_cnt = true;

        res.result = false;
    }
}

bool jane_control_class::fn_previous_mission(jane_ui::previous_mission::Request &req,
jane_ui::previous_mission::Response &res){
    log_cnt = true;

    wp_idx--;
    if (wp_idx<0){
        wp_idx++;
        UAV_control_mode = hover_wp_mode;
        // log
        loglog ="This is the first waypoint, cannot go to previous waypoint";
        log_pub_ros_info(LOG_ERROR,loglog);
        log_cnt = true;

        res.result = false;
    }
    else {
        UAV_control_mode = move_wp_mode;
//        // log
//        loglog = "Depart to previous waypoint";
//        log_pub_ros_info(LOG_NORMAL,loglog);
//        log_cnt = true;

        res.result = true;
    }
}

bool jane_control_class::fn_pause_mission(jane_ui::pause_mission::Request &req,
jane_ui::pause_mission::Response &res){
    log_cnt = true;

    pause_toggle = !pause_toggle;
    UAV_control_mode = pause_mode;
    res.result = true;
}

bool jane_control_class::fn_return_home(jane_ui::return_home::Request &req,
jane_ui::return_home::Response &res){
    log_cnt = true;

    UAV_control_mode = return_home_mode;
    res.result = true;
    start_wp = pix_poseLocal.pose.pose.position;
    end_wp = pix_home.position;
    end_wp.z = start_wp.z;
}

bool jane_control_class::fn_emergency_landing(jane_ui::emergency_landing::Request &req,
jane_ui::emergency_landing::Response &res){
    log_cnt = true;

    UAV_control_mode = emergency_mode;
    res.result = true;
}
