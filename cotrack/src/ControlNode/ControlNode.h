#ifndef CONTROLNODE_H
#define CONTROLNODE_H
#include <fstream>
#include <math.h>
#include <cmath>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>

//ros message
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

class ControlNode
{

private:
  //ros 句柄
  ros::NodeHandle nh_;
  //发布频率
  int publishFreq;
  //【发布】控制指令
  ros::Publisher drone1_cmd_pub;

  //【变量】追踪开启标志位
  bool track_flag;

  /*【状态机】
   * int state_machine
   * 0 ------> 匀速平行追踪
   * 1 ------> 环绕式追踪
   * 2 ------> time-varying formation track
   */
  int state_machine;

  /*【拓扑结构】
   * 0 ------->cyclic pursuit(default)
   * 1 ------->ring topology
   * 2 ------->all to all communication
   * int topology_flag;
   */
  int topology_flag;




  //私有变量
  double bp1_current_position[3];             //bebop 当前 xyz
  double bp1_current_yaw;                     //bebop 当前 yaw
  double bp1_target_yaw;                      //bebop 当前 yaw
  double bp1_current_v[2];                    //bebop 速度 v

  geometry_msgs::Twist bp1_cmd;


public:
  ControlNode();
  ~ControlNode();


  void Loop();
};

#endif // CONTROLNODE_H
