#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include "string.h"
#include <boost/foreach.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#define forEach BOOST_FOREACH
using namespace std;
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double dis = 0;
geometry_msgs::PoseStamped goal;
//
void odometryCallback(nav_msgs::OdometryConstPtr odom)
{

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;
  ROS_INFO_STREAM("reveive odometry");
}

void fillPathRequest(nav_msgs::GetPlan::Request &request)
{
    request.start.header.frame_id = "map";
    request.start.pose.position.x = vehicleX;
    request.start.pose.position.y = vehicleY;
    request.start.pose.orientation.w = 1;
    request.goal.header.frame_id = "map";
    request.goal.pose.position.x = goal.pose.position.x;//终点坐标
    request.goal.pose.position.y = goal.pose.position.y;
    request.goal.pose.orientation.w = 1.0;
    request.tolerance = 0.5;//如果不能到达目标，最近可到的约束
}
//路线规划结果回调
void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
{
    if(serviceClient.call(srv))
    {
        if(!srv.response.plan.poses.empty())
        {
            forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses)
            {
                ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
            }
        }
        else {
            ROS_WARN("Got empty plan");
        }
    }
    else 
    {
        ROS_ERROR("Failed to call service %s - is the robot moving?",serviceClient.getService().c_str());
    }
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc,argv,"make_plane");
    ros::NodeHandle nh;
    std::string service_name = "move_base_node/make_plane";
    ros::Publisher pub_WayPoint = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
    ros::Subscriber sub_Odom = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 10, odometryCallback);

    while (!ros::service::waitForService(service_name, ros::Duration(3.0))) 
    {
        ROS_INFO("Waiting for service move_base/make_plan to become available");
    }

    ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
    if (!serviceClient) 
    {
        ROS_FATAL("Could not initialize get plan service from %s",
        serviceClient.getService().c_str());
        return -1;
    }
    nav_msgs::GetPlan srv;
    



    int num = 0;
    dis = sqrt((goal.pose.position.x - vehicleX) * (goal.pose.position.x - vehicleX) + (goal.pose.position.y - vehicleY) * (goal.pose.position.y - vehicleY));
    if(dis < 2)
        pub_WayPoint.publish(goal);
    else
    {
        //请求服务：规划路线
        fillPathRequest(srv.request);
        if (!serviceClient) 
        {
            ROS_FATAL("Persistent service connection to %s failed",
            serviceClient.getService().c_str());
            return -1;
        }
        ROS_INFO("conntect to %s",serviceClient.getService().c_str());
        callPlanningService(serviceClient, srv);
        num = srv.response.plan.poses.size();
        pub_WayPoint.publish(srv.response.plan.poses[num/2]);
    }
    return 0;
}
