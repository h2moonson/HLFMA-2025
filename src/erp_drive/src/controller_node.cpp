#include "pure_pursuit.h"

void paramCallback(erp_drive::CtrlParamConfig &config, PurePursuit* control) {
    // ROS_INFO("velocity : %lf lookahead_distance : %lf", config.velocity, config.lookahead_distance);
    control->velocity = config.velocity;
    control->lookahead_distance = config.lookahead_distance;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit");

    ROS_INFO("controller_node init");

    PurePursuit control(argv[1][0]);

    control.l_curr_idx = 0;
    control.l_next_idx = 0;

    ros::Publisher target_point = control.nh.advertise<geometry_msgs::PoseStamped>("/rviz_target", 1);
    
    control.curr_pose.position.x = 0.0;
    control.curr_pose.position.y = 0.0;
    control.curr_pose.position.z = 0.0;
    
    control.curr_pose.orientation.x = 0.0;
    control.curr_pose.orientation.y = 0.0;
    control.curr_pose.orientation.z = 0.0;
    control.curr_pose.orientation.w = 1.0;

    ros::Rate loop_rate(control.LOOP_RATE);

    dynamic_reconfigure::Server<erp_drive::CtrlParamConfig> server;
    dynamic_reconfigure::Server<erp_drive::CtrlParamConfig>::CallbackType f;
    
    f = boost::bind(&paramCallback, _1, &control);
    server.setCallback(f);

    while(ros::ok()){
        ros::spinOnce(); // 콜백 함수 호출

        if(control.is_local_path_set){
            ROS_INFO_STREAM("Controller (local) : Curr_Idx : " << control.l_curr_idx << ", Curr Accel : " << control.velocity);        
            
            control.calcSteering(control.l_path, control.curr_pose);

            geometry_msgs::PoseStamped rviz_target;
            rviz_target.pose.position.x = control.l_path.path[control.l_next_idx].pose.position.x;
            rviz_target.pose.position.y = control.l_path.path[control.l_next_idx].pose.position.y;
            rviz_target.pose.position.z = control.l_path.path[control.l_next_idx].pose.position.z;

            rviz_target.header.stamp = ros::Time::now();
            rviz_target.header.frame_id = "velodyne";

            target_point.publish(rviz_target);

            control.publishControlMsg();
        }

        control.l_next_idx = 0;
        loop_rate.sleep();
    }

    return 0;
}
