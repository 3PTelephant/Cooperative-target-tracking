#include "ControlNode.h"

double pi=3.1415926;


ControlNode::ControlNode()
{
  //【发布】 移动命令
  drone1_cmd_pub=nh_.advertise<geometry_msgs::Twist>("/bebop1/cmd_vel", 5);
  //控制指令发布频率
  publishFreq=30;
  ros::Rate rate(publishFreq);

}


ControlNode::~ControlNode()
{

}

ControlNode::Loop()
{
  while ( ros::ok() )
  {
    ros::spinOnce();

    //是否初始化，比如变量rou，theta等



    //第一层逻辑：是否追踪

    if (track_flag==0)
    {
      //hover
    }

    else
    {
      //几种追踪方式

      //匀速追踪的控制率
      if (state_machine==0)
      {
        //根据拓扑结构生成相应的控制指令
        if (topology_flag==0)
        {
          //单无人机追踪控制率---->利用二阶积分器模型
          double ;//


          //偏航角的控制率------->单独一个通道去对准


          double yaw_command,yaw_relative;
          yaw_relative=0;
          yaw_command=(bp1_target_yaw-bp1_current_yaw)*0.2+yaw_relative;
          if (yaw_command>pi)
          {
            bp1_cmd.angular.z=(yaw_command-2*pi);
          }
          else if (yaw_command<=-pi)
              {
               bp1_cmd.angular.z=(yaw_command+2*pi);
              }
              else
              {
                bp1_cmd.angular.z=(yaw_command);
              }

          //协同量



        drone1_cmd_pub.publish(bp1_cmd);




        }
        if (topology_flag==1)
        {

        }
        if (topology_flag==2)
        {

        }


      }

      //圆形追踪
      if (state_machine==1)
      {

        //根据拓扑结构生成相应的控制指令
        if (topology_flag==0)
        {
          //单无人机追踪控制率---->利用极坐标系的二阶积分器模型



          //偏航角的控制率------->单独一个通道去对准


          double yaw_command,yaw_relative;
          yaw_command=(bp1_target_yaw-bp1_current_yaw)*0.2+yaw_relative;
          if (yaw_command>pi)
          {
            bp1_cmd_vel.angular.z=(yaw_command-2*pi);
          }
          else if (yaw_command<=-pi)
              {
               bp1_cmd_vel.angular.z=(yaw_command+2*pi);
              }
              else
              {
                bp1_cmd_vel.angular.z=(yaw_command);
              }


          //协同量

        }


      }


      //time-varying formation flight
      if (state_machine==2)
      {


      }


    }


  }
}
