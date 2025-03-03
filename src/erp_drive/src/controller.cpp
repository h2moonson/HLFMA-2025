#include "controller.h"

Controller::Controller(char control_target):
    optimal_throttle(),
    is_path_set(false), is_pose_set(false), is_local_path_set(false), g_curr_idx(0) {
        this->is_erp = control_target == 'e'; // e => erp (true), m => morai (false)

        // this->erp_drive_pub = nh.advertise<race::drive_values>("control_value", 1);
        // this->morai_drive_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

        this->curr_waypoint_pub = nh.advertise<std_msgs::Int16>("/curr_idx", 1);

        this->stop_event_srv_server = nh.advertiseService("stop_event", &Controller::getStopEvent, this);
    
        this->g_path_sub = nh.subscribe("/global_path", 1, &Controller::globalPathCallback, this);
        this->l_path_sub = nh.subscribe("/local_path", 1, &Controller::localPathCallback, this);
        this->l_path_done_sub = nh.subscribe("/local_path_done", 1, &Controller::localPathDoneCallback, this); 

        this->curr_pose_sub = nh.subscribe("/current_pose", 1, &Controller::currentPoseCallback, this);
    }

void Controller::globalPathCallback(const erp_drive::PathReference& g_path){
    this->g_path = g_path;
    this->g_path_size = g_path.path.size();
    this->is_path_set = true;
}

void Controller ::localPathCallback(const erp_drive::PathReference& l_path){
    // ROS_INFO("localPathCallback")

    this->l_path = l_path;
    this->l_path_size = l_path.path.size();

    double start_x = this->l_path.path.begin()->pose.position.x;
    double start_y = this->l_path.path.begin()->pose.position.y;
    
    double end_x = this->l_path.path.rbegin()->pose.position.x;
    double end_y = this->l_path.path.rbegin()->pose.position.y;
    
    double slop = atan2(end_y - start_y, end_x - start_x);

    ROS_INFO_STREAM("slop : " << slop);

    this->l_curr_idx = 0; // path가 갱신되었기 때문
    this->is_local_path_set = true;
}

void Controller::localPathDoneCallback(const std_msgs::Int16& siganl){
    // ROS_INFO("Controller::localPathDoneCallback() : Local Path Done, Global Path Again")
    this->is_local_path_set = false;
}

void Controller::currentPoseCallback(const geometry_msgs::Pose& curr_pose){
    ROS_INFO_STREAM("cuurentPoseCallback : get curr pose");
    this->curr_pose = curr_pose;
    this->is_pose_set = true;
}

int Controller::calcGlobalCurrWaypoint(const geometry_msgs::Pose& curr_pose){
    int min_idx = -1;
    double min_dist = 1e9, prev_dist = 1e9;

    for(int i = this->g_curr_idx; i < this->g_path_size; i++){
        double curr_dist = this->getDistance(this->g_path.path[i].pose, curr_pose);
        if(prev_dist < curr_dist) break;
        
        if(min_dist > curr_dist) {
            min_dist = curr_dist;
            min_idx = i;
        }

        prev_dist = curr_dist;
    }

    this->g_curr_idx = min_idx;
    
    std_msgs::Int16 msg;
    msg.data = this->g_curr_idx;
    this->curr_waypoint_pub.publish(msg); //planner에서 활용

    return this->g_curr_idx;
}

int Controller::calcLocalCurrWaypoint(const erp_drive::PathReference& local_path, const geometry_msgs::Pose& curr_pose){
    int min_idx = -1;
    double min_dist = 1e9, prev_dist = 1e9;

    this->l_curr_idx = 0;
    for(int i = 0; i < local_path.path.size(); i++){
        double curr_dist = this->getDistance(local_path.path[i].pose, local_path.path[i].pose);
        if(prev_dist < curr_dist) break;
        
        if(min_dist > curr_dist) {
            min_dist = curr_dist;
            min_idx = i;
        }

        prev_dist = curr_dist;
    }

    this->l_curr_idx = min_idx;
    
    std_msgs::Int16 msg;
    msg.data = this->l_curr_idx;
    this->curr_waypoint_pub.publish(msg); //planner에서 활용
    
    return this->l_curr_idx;
}

bool Controller::getStopEvent(erp_drive::GetStopEvent::Request &req, erp_drive::GetStopEvent::Response &res){
    if(req.occurEvent){
        this->optimal_throttle.shouldStop = true;
        this->optimal_throttle.start_time = ros::Time::now();
        this->optimal_throttle.start_pose = this->curr_pose;

        res.success = true;
    }
    
    return true;
}

void Controller::publishControlMsg(){
    ROS_INFO("Im dead");

    // // ERP로 주행 시 실행되는 블럭
    // if(is_erp){
    //     race::drive_values msg;
    //     msg.throttle = this->velocity;
    //     // 주의! deg 단위임
    //     msg.steering = this->steering * (180 / M_PI) * (-1);
    //     msg.brake = 0.;

    //     this->erp_drive_pub.publish(msg);
    // }

    // // 모라이로 주행 시 실행되는 블럭
    // else {
    //     morai_msgs::CtrlCmd msg;
    //     msg.accel = this->velocity / 20.0;
    //     // 주의! rad 단위임
    //     msg.steering = this->steering;
    //     msg.brake = 0.0;

    //     if(this->velocity < 0.5){
    //         msg.brake = 1.0;
    //     }

    //     this->morai_drive_pub.publish(msg);    
    // }
}
