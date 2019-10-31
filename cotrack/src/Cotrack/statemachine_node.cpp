#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "iostream"
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "statemachine_node");
  ros::NodeHandle nh("~");

  ros::Publisher statemachine_pub = nh.advertise<std_msgs::Int32>("/statemachine", 1000);
  ros::Publisher reconfigure_pub = nh.advertise<std_msgs::Int32>("/reconfigure", 1000);

  ros::Publisher bp4_land_pub   = nh.advertise<std_msgs::Empty>("/bebop4/land", 5);            // 发布 降落命令
  ros::Publisher bp3_land_pub    = nh.advertise<std_msgs::Empty>("/bebop3/land", 5);    // 发布 移动命令
  ros::Publisher bp2_land_pub    = nh.advertise<std_msgs::Empty>("/bebop2/land", 5);    // 发布 移动命令
  ros::Publisher bp1_land_pub    = nh.advertise<std_msgs::Empty>("/bebop1/land", 5);    // 发布 移动命令

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Int32 msg;
    int input;
    cout<<"please input the machine state:"<<endl;
    cin>>input;
    switch (input)
    {
    case 1:
      msg.data=input;
      statemachine_pub.publish(msg);
      break;

    case 2:
      msg.data=input;
      statemachine_pub.publish(msg);
      break;

    case 3:
      bp1_land_pub.publish(std_msgs::Empty());
      bp2_land_pub.publish(std_msgs::Empty());
      bp3_land_pub.publish(std_msgs::Empty());
      bp4_land_pub.publish(std_msgs::Empty());
      bp1_land_pub.publish(std_msgs::Empty());
      bp2_land_pub.publish(std_msgs::Empty());
      bp3_land_pub.publish(std_msgs::Empty());
      bp4_land_pub.publish(std_msgs::Empty());
      break;

    case 4:
      bp1_land_pub.publish(std_msgs::Empty());
      bp1_land_pub.publish(std_msgs::Empty());
      msg.data=1;
      reconfigure_pub.publish(msg);
      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
