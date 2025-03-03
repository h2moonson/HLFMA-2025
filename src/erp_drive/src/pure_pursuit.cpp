#include "pure_pursuit.h"

PurePursuit::PurePursuit(char control_target):
    Controller(control_target), is_finish(false), g_next_idx(0), lookahead_distance(10.) {}

geometry_msgs::Pose PurePursuit::calcRelativeCoordinateAboutCurr(const geometry_msgs::Pose& pt, const geometry_msgs::Pose& curr_pose){
    tf::Transform inverse;
    tf::poseMsgToTF(curr_pose, inverse);
    tf::Transform transform = inverse.inverse();

    tf::Pose p;
    poseMsgToTF(pt, p);

    tf::Pose tf_p = transform * p;
    geometry_msgs::Pose ret;
    poseTFToMsg(tf_p, ret);

    return ret;
}

void PurePursuit::calcSteering(const erp_drive::PathReference& local_path, const geometry_msgs::Pose& curr_pose) {
    geometry_msgs::Pose curr_waypoint;
    geometry_msgs::Pose next_waypoint;

    if(!this->is_local_path_set)
        return;
    
    calcLocalNextWaypoint(local_path, curr_pose);

    curr_waypoint.position.x = 0.0;
    curr_waypoint.position.y = 0.0;
    curr_waypoint.position.z = 0.0;
    
    curr_waypoint.orientation.x = 0.0;
    curr_waypoint.orientation.y = 0.0;
    curr_waypoint.orientation.z = 0.0;
    curr_waypoint.orientation.w = 1.0;

    next_waypoint = local_path.path.at(this->l_next_idx).pose;

    double denominator = pow(getDistance(next_waypoint, curr_pose), 2);
    double numerator = 2 * next_waypoint.position.y;

    if(denominator != 0)
        this->steering = atan(this->WHEEL_BASE * numerator / denominator);
    
    else
        this->steering = 0.;
}

int PurePursuit::calcGlobalNextWaypoint(const geometry_msgs::Pose& curr_pose) {
    if(this->g_path_size == 0){
        ROS_ERROR("calcNextWaypoint(): no path!");
        return this->g_next_idx = -1;
    }

    this->calcGlobalCurrWaypoint(curr_pose);

    for(int i = this->g_curr_idx; i < this->g_path_size - 1; i++){
        if(i == this->g_path_size - 2){
            this->is_finish = true;
            return this->g_next_idx = i + 1;
        }

        if(getDistance(this->g_path.path[i].pose, curr_pose) >= this->lookahead_distance) {
            return this->g_next_idx = i;
        }
    }

    return this->g_next_idx;
}

int PurePursuit::calcLocalNextWaypoint(const erp_drive::PathReference& local_path, const geometry_msgs::Pose& curr_pose) {
    if(this->l_path_size == 0){
        ROS_ERROR("calcNextWaypoint(): no path!");
        return this->l_next_idx = 0;
    }
    
    for(this->l_next_idx = 0; this->l_next_idx < local_path.path.size() - 1; this->l_next_idx++){
        if(this->l_next_idx == local_path.path.size() - 2)
            return this->l_next_idx += 1;

        if(getDistance(local_path.path[this->l_next_idx].pose, curr_pose) >= this->lookahead_distance)
            return this->l_next_idx;

        //ROS_INFO_STREAM("l_curr_idx : " << l_curr_idx << ", l_next_idx : " << l_next_idx);
    }

    return this->l_next_idx;
}