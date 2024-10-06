#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include "vector"
#include "usr_fun/refree.h"
#include "usr_fun/control.h"
#include "local_goal.h"
#include  <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include "iostream"
using namespace std;

#define FRAME_REFREE_HEADER 0xa5
#define FRAME_REFREE_TAILER 0xff

#define MAP_X_MAX 8.2
#define MAP_X_MIN -8
#define MAP_Y_MAX 25
#define MAP_Y_MIN -6

#define QIANSHAO_X 3.1
#define QIANSHAO_Y 9.73
CamsenseConfig* camsense_config;

gimbal_send_msg * gimbal_send = new gimbal_send_msg();      
char DATA_TX_BUFF[sizeof(gimbal_send_msg)];
double roll, pitch, yaw;
serial::Serial gimbal_serial;

// usr_fun::control control_msg;       
//////////////////
geometry_msgs::PoseStamped goal;
geometry_msgs::Pose current_pose;
geometry_msgs::Twist cmd_vel;             

//////////////////
unsigned char rx_buffer[sizeof(usr_fun::refree)];

void poseCallback(geometry_msgs::Pose::ConstPtr msg){
    current_pose.position = msg->position;
}

void cmd_velCallback(geometry_msgs::TwistConstPtr msg)
{
    // ROS_INFO_STREAM("cmd_vel SENDING \n");
    gimbal_send->v_x = msg->linear.x;
    gimbal_send->v_y = -msg->linear.y;
    gimbal_send->w_z = -msg->angular.z;
}


tf2_msgs::TFMessage tf_msg;
geometry_msgs::PointStamped point_odom;
geometry_msgs::PointStamped point_map;
Eigen::Matrix<double, 4, 4> TF;

int main(int argc, char *argv[])
{
    camsense_config = SharedMemory("camsense");

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "gimbal_serial");
    ros::NodeHandle nh;
    
    gimbal_serial.setPort("/dev/ttyACM0");
    gimbal_serial.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    gimbal_serial.setTimeout(to);
    try
    {
        gimbal_serial.open();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");           //打开串口失败，打印信息
        return -1;
    }
    
    if(gimbal_serial.isOpen())
    { 
        ROS_INFO_STREAM("Serial Port initialized. \n");         //成功打开串口，打印信息  
    }
    else
    {
        return -1;
    }

    ros::Subscriber pose_sub = nh.subscribe("/odom_base_link", 10, poseCallback);
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, cmd_velCallback);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);    
    ros::Publisher refree_pub = nh.advertise<usr_fun::refree>("refree",100);
    ros::Publisher point_map_pub = nh.advertise<geometry_msgs::PointStamped>("point_map",100);
    ros::Rate loop_rate(50);

    gimbal_send->header = FRAME_REFREE_HEADER;
    gimbal_send->tailer = FRAME_REFREE_TAILER;

    while(ros::ok())
    {
        //read port
        if(gimbal_serial.available() >= sizeof(usr_fun::refree))
        {
            std::vector<int> header_pos;
            std::vector<int> tailer_pos;
            int head_pos;
            int tail_pos;
            gimbal_serial.read(rx_buffer,sizeof(usr_fun::refree));
            // for(int i = 0;i<sizeof(usr_fun::refree);i++)
            // {
            //     printf("%02X",static_cast<unsigned char>(rx_buffer[i]));
            // }
            // printf("\n");
            for(int i = 0;i<sizeof(usr_fun::refree);i++){
                if(rx_buffer[i] == FRAME_REFREE_HEADER){
                    head_pos = i;
                }
                if(rx_buffer[i] == FRAME_REFREE_TAILER){
                    tail_pos = i;
                }
            }
            unsigned char temp_buffer[sizeof(usr_fun::refree)];
            for(int i=0;i<sizeof(usr_fun::refree);i++){
                temp_buffer[i] = rx_buffer[(head_pos + i)%sizeof(usr_fun::refree)];
            }
            memcpy(&refree_msg,temp_buffer,sizeof(usr_fun::refree));
            ROS_INFO("publish refree");
            refree_pub.publish(refree_msg);
            //////////////
            //goal.pose
        }
        ros::Duration(10.0).sleep();
            //right up
            goal.pose.position.z = 0;
            goal.pose.position.y = 2.302;
            goal.pose.position.x = 1.53455;
            goal.pose.orientation.z = 0.307192;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.x = 0;
            goal.pose.orientation.w = 0.95164;
            goal_pub.publish(goal);
            ROS_INFO("publish goal");
            ros::Duration(10.0).sleep();

            //right down
            goal.pose.position.z = 0.0;
            goal.pose.position.y = 0.49690;
            goal.pose.position.x = 1.0142;
            goal.pose.orientation.z = 0.50615;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.x = 0;
            goal.pose.orientation.w = 0.862442;
            goal_pub.publish(goal);
            ROS_INFO("publish goal");
            ros::Duration(10.0).sleep();

            //left up
            goal.pose.position.z = 0;
            goal.pose.position.y = 2.27644;
            goal.pose.position.x = -1.6044;
            goal.pose.orientation.z = 0.23046;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.x = 0;
            goal.pose.orientation.w = 0.97308;
            goal_pub.publish(goal);
            ROS_INFO("publish goal");
            ros::Duration(10.0).sleep();

            //left down
            goal.pose.position.z = 0;
            goal.pose.position.y = 0.379033;
            goal.pose.position.x = -1.54305;
            goal.pose.orientation.z = 0.269;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.x = 0;
            goal.pose.orientation.w = 0.96311;
            goal_pub.publish(goal);
            ROS_INFO("publish goal");
            ros::Duration(10.0).sleep();

            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();

        if( current_pose.position.x >= MAP_X_MAX || current_pose.position.x <= MAP_X_MIN ||
            current_pose.position.y >= MAP_Y_MAX || current_pose.position.y <= MAP_Y_MIN ){
            gimbal_send->v_x = 0;
            gimbal_send->v_y = 0;
            gimbal_send->w_z = 0;
        }
        memcpy(DATA_TX_BUFF,gimbal_send,sizeof(gimbal_send_msg));
        uint8_t * a = (uint8_t *)DATA_TX_BUFF;
        gimbal_serial.write((const uint8_t *)DATA_TX_BUFF,sizeof(gimbal_send_msg));    

        point_map_pub.publish(point_map);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
