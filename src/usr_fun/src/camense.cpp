#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "usr_fun/camense.h"
#include "usr_fun/refree.h"
#include "local_goal.h"

CamsenseConfig* camsense_config;

void camenseCallback(geometry_msgs::Pose::ConstPtr msg)
{
    camsense_config->orientation_w = msg->orientation.w;
    camsense_config->orientation_x = msg->orientation.x;
    camsense_config->orientation_y = msg->orientation.y;
    camsense_config->orientation_z = msg->orientation.z;
    camsense_config->position_x = msg->position.x;
    camsense_config->position_y = msg->position.y;
    camsense_config->position_z = msg->position.z;
    ROS_INFO("the current pose has been published succcessfully!");
}
void refreeCallback(usr_fun::refree::ConstPtr msg)
{
    camsense_config->nav_at_aim_yaw = msg->diff_angle;
    camsense_config->curr_aim_yaw = msg->curr_yaw;
    switch (msg->team) {
        case 1:
            camsense_config->receive_config = CAMSENSE_COLOR_BLUE;
            break;
        case 0:
            camsense_config->receive_config = CAMSENSE_COLOR_RED;
            break;
    }
}

int main(int argc, char *argv[])
{
    camsense_config = SharedMemory("camsense");

    ros::init(argc,argv,"camense_sensor");
    ros::NodeHandle nh;
    ros::Subscriber map_baselink_sub=nh.subscribe<geometry_msgs::Pose>("/odom_base_link",10,camenseCallback);
    ros::Subscriber refree_sub = nh.subscribe<usr_fun::refree>("refree",10,refreeCallback);
    ros::Rate rate(5);
    
    while(ros::ok()) {
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
