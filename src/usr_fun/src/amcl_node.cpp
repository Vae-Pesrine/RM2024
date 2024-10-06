#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

uint32_t count=0;
tf2_msgs::TFMessage tf_map_odom;
geometry_msgs::Pose tf_map_baselink;

Eigen::Matrix<double, 4, 4> TF;
Eigen::Matrix<double,4,4> q1;
Eigen::Matrix<double,4,1> q2; 
Eigen::Matrix<double,4,1> q_result; 

int pub_flag = 0,kill_flag = 0;


void tf_quater2rotate(
    Eigen::Matrix<double, 4, 4>& matrix,
    const double qx, const double qy, const double qz, const double qw,
    double x, double y, double z
) {
    double m00 = 1 - 2 * qy * qy - 2 * qz * qz;
    double m01 = 2 * qx * qy - 2 * qz * qw;
    double m02 = 2 * qx * qz + 2 * qy * qw;
    double m10 = 2 * qx * qy + 2 * qz * qw;
    double m11 = 1 - 2 * qx * qx - 2 * qz * qz;
    double m12 = 2 * qy * qz - 2 * qx * qw;
    double m20 = 2 * qx * qz - 2 * qy * qw;
    double m21 = 2 * qy * qz + 2 * qx * qw;
    double m22 = 1 - 2 * qx * qx - 2 * qy * qy;
    matrix << m00, m01, m02, x,
              m10, m11, m12, y,
              m20, m21, m22, z,
              0,   0,   0,   1;
}

void amclCallback(tf2_msgs::TFMessageConstPtr msg)
{
    if(msg->transforms[0].child_frame_id == "odom" && msg->transforms[0].header.frame_id == "map" && count < 150)
    {
        tf_map_odom.transforms = msg->transforms;
        ROS_INFO("receive tf map-odom");
    }
}

void odomCallback(nav_msgs::Odometry::ConstPtr msg){
    Eigen::Vector4d p;
    p << msg->pose.pose.position.x,
         msg->pose.pose.position.y,
         msg->pose.pose.position.z,
         1;
    q2 << msg->pose.pose.orientation.w,
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z;
    Eigen::Vector4d m = TF * p;

    tf_map_baselink.position.x = m[0];
    tf_map_baselink.position.y = m[1];
    tf_map_baselink.position.z = m[2];
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"amcl_node");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    ros::Subscriber amcl_sub = nh.subscribe("/tf", 10, amclCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom" ,10, odomCallback);
    ros::Publisher  tf_pub_map_odom = nh.advertise<tf2_msgs::TFMessage>("/tf",10);
    ros::Publisher  tf_pub_map_baselink = nh.advertise<geometry_msgs::Pose>("odom_base_link",10);
    ros::Rate loop_rate(20);
    // ros::Rate sub_rate(20);
    // ros::Rate pub_rate(10);

    while(ros::ok()){
        if(kill_flag == 0)
            count++;
        if(count >= 150 && count <= 160 && kill_flag == 0){
            pub_flag = 1;

            q1 <<  tf_map_odom.transforms[0].transform.rotation.w, -tf_map_odom.transforms[0].transform.rotation.x, 
                   -tf_map_odom.transforms[0].transform.rotation.y, -tf_map_odom.transforms[0].transform.rotation.z,

                   tf_map_odom.transforms[0].transform.rotation.x, tf_map_odom.transforms[0].transform.rotation.w,
                   -tf_map_odom.transforms[0].transform.rotation.z,tf_map_odom.transforms[0].transform.rotation.y,

                   tf_map_odom.transforms[0].transform.rotation.y, tf_map_odom.transforms[0].transform.rotation.z,
                   tf_map_odom.transforms[0].transform.rotation.w, -tf_map_odom.transforms[0].transform.rotation.x,

                   tf_map_odom.transforms[0].transform.rotation.z, -tf_map_odom.transforms[0].transform.rotation.y,
                   tf_map_odom.transforms[0].transform.rotation.x, tf_map_odom.transforms[0].transform.rotation.w;
            tf_quater2rotate(
                TF,
                tf_map_odom.transforms[0].transform.rotation.x, tf_map_odom.transforms[0].transform.rotation.y,
                tf_map_odom.transforms[0].transform.rotation.z, tf_map_odom.transforms[0].transform.rotation.w,
                tf_map_odom.transforms[0].transform.translation.x, tf_map_odom.transforms[0].transform.translation.y, tf_map_odom.transforms[0].transform.translation.z
            );
            system("rosnode kill amcl");
        }
        if(count > 160){
            kill_flag = 1;
        }
        if(pub_flag == 1){
            tf_map_odom.transforms[0].header.stamp = ros::Time::now();
            
            q_result = q1 * q2;
            tf_map_baselink.orientation.w = q_result[0];
            tf_map_baselink.orientation.x = q_result[1];
            tf_map_baselink.orientation.y = q_result[2];
            tf_map_baselink.orientation.z = q_result[3];

            tf_pub_map_odom.publish(tf_map_odom);
            tf_pub_map_baselink.publish(tf_map_baselink);
            ROS_INFO("publish tf map-odom");
        }
        //ROS_INFO("success");
        ros::spinOnce();
        loop_rate.sleep();
        // if(pub_flag == 0) sub_rate.sleep();
        // if(pub_flag == 1) pub_rate.sleep();
    }
    return 0;
}
