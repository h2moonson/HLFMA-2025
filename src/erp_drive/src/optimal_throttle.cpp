#include "optimal_throttle.h"

double OptimalThrottle::calcLinearVelocity(geometry_msgs::Pose curr_pose)
{
    double curr_time = (ros::Time::now() - start_time).toNSec() / this->NANO_TO_SEC;
    ROS_INFO_STREAM("curr_time : " << curr_time);

    double dist_to_stop = (this->start_pose.position.x - curr_pose.position.x) * (this->start_pose.position.x - curr_pose.position.x);
    dist_to_stop = (this->start_pose.position.y - curr_pose.position.y) * (this->start_pose.position.y - curr_pose.position.y);
    dist_to_stop = sqrt(dist_to_stop);

    if(shouldStop)
        return 20.0 / 3.0 * dist_to_stop;

    return MAX_VELOCITY;
}

double OptimalThrottle::calcTanhVelocity(geometry_msgs::Pose curr_pose)
{
    double curr_time = (ros::Time::now() - start_time).toNSec() / this->NANO_TO_SEC;
    ROS_INFO_STREAM("curr_time : " << curr_time);

    double dist_to_stop = pow(this->start_pose.position.x - curr_pose.position.x, 2);
    dist_to_stop += pow(this->start_pose.position.y - curr_pose.position.y, 2);
    dist_to_stop = sqrt(dist_to_stop);

    if(shouldStop)
        return 20.0 * tanh(dist_to_stop / 7.0);

    return MAX_VELOCITY;
}

double OptimalThrottle::calcCosineVelocity()
{
    double curr_time = (ros::Time::now() - start_time).toNSec() / this->NANO_TO_SEC;

    if(shouldStop){
        if(curr_time >= 3.0) return 0.;
        return 10.0 * cos(curr_time / 0.955) + 10.0;
    }

    return MAX_VELOCITY;
}

double OptimalThrottle::calcSqrtVelocity()
{
    double curr_time = (ros::Time::now() - start_time).toNSec() / this->NANO_TO_SEC;
    ROS_INFO_STREAM("curr_time : " << curr_time);

    if(shouldStop){
        if(curr_time >= 3.0) return 0.;
        return 15 * sqrt(-curr_time + 4) / sqrt(4);
    }

    return MAX_VELOCITY;
}

