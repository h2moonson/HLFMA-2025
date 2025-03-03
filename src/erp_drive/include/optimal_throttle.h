#ifndef __OPTIMAL_THROTTLE_H__
#define __OPTIMAL_THROTTLE_H__

// ros 관련 헤더
#include <ros/ros.h>

// 좌표계 및 기하 연산 관련 헤더
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Path 관련 커스텀 메세지
#include <erp_drive/PathReference.h>
#include <erp_drive/PathReferenceElement.h>

// velocity 등 parameter 서버를 위한 헤더 
#include <dynamic_reconfigure/server.h>
#include <erp_drive/CtrlParamConfig.h>

#include <string>

class OptimalThrottle
{
public:
    const double DISTANCE = 10.0;
    const double STOP_TIME = 3.0;
    const double MAX_VELOCITY = 5.0;

    // ns를 s단위로 바꾸기 위함
    const double NANO_TO_SEC = 1000000000.0;

    bool shouldStop;

    double throttle;
    ros::Time start_time;

    geometry_msgs::Pose start_pose;

public:
    double calcSqrtVelocity();
    double calcCosineVelocity();
    double calcTanhVelocity(geometry_msgs::Pose curr_pose);
    double calcLinearVelocity(geometry_msgs::Pose curr_pose);
};

#endif