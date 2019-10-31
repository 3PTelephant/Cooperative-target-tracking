#include "StateEstimation.h"

using namespace std;


StateEstimationNode::StateEstimationNode()
{
  //发布频率
  publishFreq=20;
  //【订阅】1号机的位姿
  drone_1_state_sub=nh_.subscribe("/vrpn_client_node/bebop1/pose",2,&StateEstimationNode::drone1stateCb,this);
  //【发布】1号机状态
  drone_1_state_pub=nh_.advertise<cotrack::drone_state>("/drone_state/drone1",10);



  //【订阅】2号机的位姿
  drone_2_state_sub=nh_.subscribe("/vrpn_client_node/bebop2/pose",2,&StateEstimationNode::drone2stateCb,this);
  //【发布】2号机状态
  drone_2_state_pub=nh_.advertise<cotrack::drone_state>("/drone_state/drone2",10);


  //【订阅】3号机的位姿
  drone_3_state_sub=nh_.subscribe("/vrpn_client_node/bebop3/pose",2,&StateEstimationNode::drone3stateCb,this);
  //【发布】3号机状态
  drone_3_state_pub=nh_.advertise<cotrack::drone_state>("/drone_state/drone3",10);


  //【订阅】4号机的位姿
  drone_4_state_sub=nh_.subscribe("/vrpn_client_node/bebop4/pose",2,&StateEstimationNode::drone4stateCb,this);
  //【发布】4号机状态
  drone_4_state_pub=nh_.advertise<cotrack::drone_state>("/drone_state/drone4",10);

  //【订阅】目标的位姿
  target_state_sub=nh_.subscribe("/vrpn_client_node/target/pose",2,&StateEstimationNode::targetstateCb,this);
  //【发布】目标状态
  target_state_pub=nh_.advertise<cotrack::drone_state>("/target_state",10);



  //【订阅】相对位置
  //relative_pose_sub=nh_.subscribe("/vision/relative_position",2,&StateEstimationNode::relativeposeCb,this);

  filter_flag=1;


  //【1号机】初始化
  drone1_ok=0;

  //【2号机】初始化
  drone2_ok=0;
  //【3号机】初始化
  drone3_ok=0;
  //【4号机】初始化
  drone4_ok=0;
  // 【目标】初始化
  target_ok=0;
  target_state_init=0;

  //drone1所有参数初始化
  drone1_euler<<0,0,0;
  drone1_vel_now<<0,0,0;
  drone1_vel_1<<0,0,0;
  drone1_vel_2<<0,0,0;
  drone1_vel_3<<0,0,0;
  drone1_vel_4<<0,0,0;

  //drone2所有参数初始化
  drone2_euler<<0,0,0;
  drone2_vel_now<<0,0,0;
  drone2_vel_1<<0,0,0;
  drone2_vel_2<<0,0,0;
  drone2_vel_3<<0,0,0;
  drone2_vel_4<<0,0,0;

  //drone3所有参数初始化
  drone3_euler<<0,0,0;
  drone3_vel_now<<0,0,0;
  drone3_vel_1<<0,0,0;
  drone3_vel_2<<0,0,0;
  drone3_vel_3<<0,0,0;
  drone3_vel_4<<0,0,0;

  //drone4所有参数初始化
  drone4_euler<<0,0,0;
  drone4_vel_now<<0,0,0;
  drone4_vel_1<<0,0,0;
  drone4_vel_2<<0,0,0;
  drone4_vel_3<<0,0,0;
  drone4_vel_4<<0,0,0;

  Q<<0.01*0.01*Eigen::MatrixXd::Identity(5,5);
  R<<0.05*0.05*Eigen::MatrixXd::Identity(3,3);
//  R(2,2)=0.01*0.01;
  cout<<"R3"<<R(2,2)<<endl;
  P<<Eigen::MatrixXd::Identity(5,5);
  H<<1,0,0,0,0,
     0,1,0,0,0,
     0,0,1,0,0;
  n_2pi=0;
  pi=3.1415926;

}
StateEstimationNode::~StateEstimationNode()
{

}
void StateEstimationNode::Loop()
{
  ros::Rate pub_rate(publishFreq);
  while (nh_.ok())
  { 
    ros::spinOnce();

    
    pub_rate.sleep();
  }

}
void StateEstimationNode::drone1stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg1)
{
  drone1_point[0] = msg1->pose.position.x;
  drone1_point[1] = msg1->pose.position.y;
  drone1_point[2] = msg1->pose.position.z;
  drone1_q.w()=msg1->pose.orientation.w;
  drone1_q.x()=msg1->pose.orientation.x;
  drone1_q.y()=msg1->pose.orientation.y;
  drone1_q.z()=msg1->pose.orientation.z;

  quaternion_2_euler(drone1_q,drone1_euler);
  if (drone1_ok==false)
  {
    drone1_lastpoint=drone1_point;
    drone1_lasttime=msg1->header.stamp;
    drone1_ok=true;
  }
  else
  {
    drone1_nowtime=msg1->header.stamp;
    Eigen::Vector3d vel_now;
    //加上判断两桢之间的时间 不满足条件直接break
    if ( (drone1_nowtime - drone1_lasttime).toSec() <0.04)
    return;
    else
    {

      vel_now = ( drone1_point - drone1_lastpoint ) / ( drone1_nowtime - drone1_lasttime ).toSec( );


      //----------- velocity filter-------------------//
      drone1_vel_now=vel_now;
      drone1_vel_now=(drone1_vel_now+drone1_vel_1+drone1_vel_2+drone1_vel_3+drone1_vel_4)*0.2;
      drone1_vel_4=drone1_vel_3;
      drone1_vel_3=drone1_vel_2;
      drone1_vel_2=drone1_vel_1;
      drone1_vel_1=drone1_vel_now;

      //----------- state publish  --------------------//
      //定义一个带有 position velocity yaw多个变量的
      cotrack::drone_state drone1_state;
      drone1_state.pos_x=drone1_point[0];
      drone1_state.pos_y=drone1_point[1];
      drone1_state.pos_z=drone1_point[2];
      drone1_state.vel_x=drone1_vel_now[0];
      drone1_state.vel_y=drone1_vel_now[1];
      drone1_state.vel_z=drone1_vel_now[2];
      drone1_state.yaw=drone1_euler[2];
      drone1_state.header=msg1->header;
      drone_1_state_pub.publish(drone1_state);

      //----------- state update  -------------------- //
      drone1_lastpoint=drone1_point;
      drone1_lasttime= drone1_nowtime;
    }

  }

}
void StateEstimationNode::drone2stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg2)
{
    drone2_point[0] = msg2->pose.position.x;
    drone2_point[1] = msg2->pose.position.y;
    drone2_point[2] = msg2->pose.position.z;
    drone2_q.w()=msg2->pose.orientation.w;
    drone2_q.x()=msg2->pose.orientation.x;
    drone2_q.y()=msg2->pose.orientation.y;
    drone2_q.z()=msg2->pose.orientation.z;

    quaternion_2_euler(drone2_q,drone2_euler);
    if (drone2_ok==false)
    {
      drone2_lastpoint=drone2_point;
      drone2_lasttime=msg2->header.stamp;
      drone2_ok=true;
    }
    else
    {
      drone2_nowtime=msg2->header.stamp;
      Eigen::Vector3d vel_now;
      //加上判断两桢之间的时间 不满足条件直接break
      if ( (drone2_nowtime - drone2_lasttime).toSec() <0.04)
      return;
      else
      {

        vel_now = ( drone2_point - drone2_lastpoint ) / ( drone2_nowtime - drone2_lasttime ).toSec( );


        //----------- velocity filter-------------------//
        drone2_vel_now=vel_now;
        drone2_vel_now=(drone2_vel_now+drone2_vel_1+drone2_vel_2+drone2_vel_3+drone2_vel_4)*0.2;
        drone2_vel_4=drone2_vel_3;
        drone2_vel_3=drone2_vel_2;
        drone2_vel_2=drone2_vel_1;
        drone2_vel_1=drone2_vel_now;

        //----------- state publish  --------------------//
        //定义一个带有 position velocity yaw多个变量的
        cotrack::drone_state drone2_state;
        drone2_state.pos_x=drone2_point[0];
        drone2_state.pos_y=drone2_point[1];
        drone2_state.pos_z=drone2_point[2];
        drone2_state.vel_x=drone2_vel_now[0];
        drone2_state.vel_y=drone2_vel_now[1];
        drone2_state.vel_z=drone2_vel_now[2];
        drone2_state.yaw=drone2_euler[2];
        drone2_state.header=msg2->header;
        drone_2_state_pub.publish(drone2_state);

        //----------- state update  -------------------- //
        drone2_lastpoint=drone2_point;
        drone2_lasttime= drone2_nowtime;
      }

    }
}
void StateEstimationNode::drone3stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg3)
{
    drone3_point[0] = msg3->pose.position.x;
    drone3_point[1] = msg3->pose.position.y;
    drone3_point[2] = msg3->pose.position.z;
    drone3_q.w()=msg3->pose.orientation.w;
    drone3_q.x()=msg3->pose.orientation.x;
    drone3_q.y()=msg3->pose.orientation.y;
    drone3_q.z()=msg3->pose.orientation.z;

    quaternion_2_euler(drone3_q,drone3_euler);
    if (drone3_ok==false)
    {
      drone3_lastpoint=drone3_point;
      drone3_lasttime=msg3->header.stamp;
      drone3_ok=true;
    }
    else
    {
      drone3_nowtime=msg3->header.stamp;
      Eigen::Vector3d vel_now;
      //加上判断两桢之间的时间 不满足条件直接break
      if ( (drone3_nowtime - drone3_lasttime).toSec() <0.04)
      return;
      else
      {

        vel_now = ( drone3_point - drone3_lastpoint ) / ( drone3_nowtime - drone3_lasttime ).toSec( );


        //----------- velocity filter-------------------//
        drone3_vel_now=vel_now;
        drone3_vel_now=(drone3_vel_now+drone3_vel_1+drone3_vel_2+drone3_vel_3+drone3_vel_4)*0.2;
        drone3_vel_4=drone3_vel_3;
        drone3_vel_3=drone3_vel_2;
        drone3_vel_2=drone3_vel_1;
        drone3_vel_1=drone3_vel_now;

        //----------- state publish  --------------------//
        //定义一个带有 position velocity yaw多个变量的
        cotrack::drone_state drone3_state;
        drone3_state.pos_x=drone3_point[0];
        drone3_state.pos_y=drone3_point[1];
        drone3_state.pos_z=drone3_point[2];
        drone3_state.vel_x=drone3_vel_now[0];
        drone3_state.vel_y=drone3_vel_now[1];
        drone3_state.vel_z=drone3_vel_now[2];
        drone3_state.yaw=drone3_euler[2];
        drone3_state.header=msg3->header;
        drone_3_state_pub.publish(drone3_state);

        //----------- state update  -------------------- //
        drone3_lastpoint=drone3_point;
        drone3_lasttime= drone3_nowtime;
      }

    }

}
void StateEstimationNode::drone4stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg4)
{
    drone4_point[0] = msg4->pose.position.x;
    drone4_point[1] = msg4->pose.position.y;
    drone4_point[2] = msg4->pose.position.z;
    drone4_q.w()=msg4->pose.orientation.w;
    drone4_q.x()=msg4->pose.orientation.x;
    drone4_q.y()=msg4->pose.orientation.y;
    drone4_q.z()=msg4->pose.orientation.z;

    quaternion_2_euler(drone4_q,drone4_euler);
    if (drone4_ok==false)
    {
      drone4_lastpoint=drone4_point;
      drone4_lasttime=msg4->header.stamp;
      drone4_ok=true;
    }
    else
    {
      drone4_nowtime=msg4->header.stamp;
      Eigen::Vector3d vel_now;
      //加上判断两桢之间的时间 不满足条件直接break
      if ( (drone4_nowtime - drone4_lasttime).toSec() <0.04)
      return;
      else
      {

        vel_now = ( drone4_point - drone4_lastpoint ) / ( drone4_nowtime - drone4_lasttime ).toSec( );

        //----------- velocity filter-------------------//
        drone4_vel_now=vel_now;
        drone4_vel_now=(drone4_vel_now+drone4_vel_1+drone4_vel_2+drone4_vel_3+drone4_vel_4)*0.2;
        drone4_vel_4=drone4_vel_3;
        drone4_vel_3=drone4_vel_2;
        drone4_vel_2=drone4_vel_1;
        drone4_vel_1=drone4_vel_now;

        //----------- state publish  --------------------//
        //定义一个带有 position velocity yaw多个变量的
        cotrack::drone_state drone4_state;
        drone4_state.pos_x=drone4_point[0];
        drone4_state.pos_y=drone4_point[1];
        drone4_state.pos_z=drone4_point[2];
        drone4_state.vel_x=drone4_vel_now[0];
        drone4_state.vel_y=drone4_vel_now[1];
        drone4_state.vel_z=drone4_vel_now[2];
        drone4_state.yaw=drone4_euler[2];
        drone4_state.header=msg4->header;
        drone_4_state_pub.publish(drone4_state);

        //----------- state update  -------------------- //
        drone4_lastpoint=drone4_point;
        drone4_lasttime= drone4_nowtime;
      }

    }

}
void StateEstimationNode::targetstateCb(const geometry_msgs::PoseStamped::ConstPtr& msg5)
{
    target_point[0] = msg5->pose.position.x;
    target_point[1] = msg5->pose.position.y;
    target_point[2] = msg5->pose.position.z;
    target_q.w()=msg5->pose.orientation.w;
    target_q.x()=msg5->pose.orientation.x;
    target_q.y()=msg5->pose.orientation.y;
    target_q.z()=msg5->pose.orientation.z;
    //加噪声上滤波算法
    quaternion_2_euler(target_q,target_euler);

    if (target_ok==false)
    {

      target_lastpoint=target_point;
      target_lasteuler=target_euler;
      target_vel_now<<0,0,0;
      target_vel_linear=0;
      target_vel_angular=0;
      target_lasttime=msg5->header.stamp;
      target_ok=true;
    }
    else
    {
      if(target_state_init==false)
      {
        target_nowtime=msg5->header.stamp;
        if ( (target_nowtime - target_lasttime).toSec() <0.04)
        return;
        else
        {
          vel_now = ( target_point - target_lastpoint ) / ( target_nowtime - target_lasttime ).toSec( );
          target_vel_linear=sqrt(vel_now[0]*vel_now[0]+vel_now[1]*vel_now[1]);
          if(target_euler[2]*target_lasteuler[2]<0&&target_euler[2]<-3)
          {
            n_2pi+=1;
          }
          cout<<"target_euler:"<<target_euler[2]<<";n_2pi:"<<n_2pi<<endl;

          target_vel_angular=(target_euler[2]-target_lasteuler[2])/( target_nowtime - target_lasttime ).toSec( );

          //-----最重要的变量------
          target_state<<target_point[0],target_point[1],target_euler[2],target_vel_linear,target_vel_angular;

          //----------- state update  -------------------- //
          target_lastpoint=target_point;
          target_lasteuler=target_euler;
          target_lasttime= target_nowtime;
          target_state_init=true;
        }
      }
      else
      {
        target_nowtime=msg5->header.stamp;
        if ( (target_nowtime - target_lasttime).toSec() <0.04)
        return;
        else
        {

         double dt;
         dt=(target_nowtime - target_lasttime).toSec();
         if(target_euler[2]*(target_lasteuler[2]-2*n_2pi*pi)<0&&target_euler[2]<-2.5)
         {
           n_2pi+=1;
         }
         target_euler[2]=target_euler[2]+2*n_2pi*pi;
         cout<<"target_euler:"<<target_euler[2]<<";n_2pi:"<<n_2pi<<endl;
         //cout<<"der_angularspeed:"<<(target_euler[2]-target_lasteuler[2])/dt;
         //----------------predict------------------------//
         Jacb<<1,0,-target_vel_linear*sin(target_lasteuler[2])*dt,0,cos(target_lasteuler[2])*dt,
                0,1,target_vel_linear*cos(target_lasteuler[2])*dt,0,sin(target_lasteuler[2])*dt,
                0,0,1,dt,0,
                0,0,0,1,0,
                0,0,0,0,1;
         prect_state=Jacb*target_state;

         P=Jacb*P*Jacb.transpose()+Q;
         //--------------correct--------------------------//
         K=P*H.transpose()*(H*P*H.transpose()+R).inverse();
         prect_part_state<<prect_state[0],prect_state[1],prect_state[2];
         observe_state<<target_point[0],target_point[1],target_euler[2];
         correct_state=prect_state+K*(observe_state-prect_part_state);
         P=P-K*H*P;


        //----------- state update  -------------------- //
        target_lastpoint=target_point;
        target_lasteuler=target_euler;
        target_lasteuler[2]=correct_state[2];
        target_vel_linear=correct_state[4];
        target_vel_angular=correct_state[3];
        target_lasttime= target_nowtime;
        target_state=correct_state;

        //----------- state publish  --------------------//
        //定义一个带有 position velocity yaw多个变量的

        cotrack::drone_state target_state_msg;
        target_state_msg.pos_x=correct_state[0];
        target_state_msg.pos_y=correct_state[1];
        target_state_msg.pos_z=target_point[2];
        target_state_msg.vel_x=correct_state[3];
        target_state_msg.vel_y=correct_state[4];
        target_state_msg.vel_z=0;
        target_state_msg.yaw=correct_state[2];
        target_state_msg.header=msg5->header;
        target_state_pub.publish(target_state_msg);
        //解算加速度
        //a=w*v; 朝向为theta+pi/2;
        //a_x=cos
        //a_y=sin
      }



      }
    }
}
//void StateEstimationNode::targetstateCb(const geometry_msgs::PoseStamped::ConstPtr& msg5)
//{
//    target_point[0] = msg5->pose.position.x;
//    target_point[1] = msg5->pose.position.y;
//    target_point[2] = msg5->pose.position.z;
//    target_q.w()=msg5->pose.orientation.w;
//    target_q.x()=msg5->pose.orientation.x;
//    target_q.y()=msg5->pose.orientation.y;
//    target_q.z()=msg5->pose.orientation.z;
//    //加噪声上滤波算法
//    quaternion_2_euler(target_q,target_euler);
//    if (target_ok==false)
//    {
//      target_lastpoint=target_point;
//      target_velocity<<0,0,0;
//      target_lasttime=msg5->header.stamp;
//      target_ok=true;
//    }
//    else
//    {
//      target_nowtime=msg5->header.stamp;
//      Eigen::Vector3d vel_now;
//      //加上判断两桢之间的时间 不满足条件直接break
//      if ( (target_nowtime - target_lasttime).toSec() <0.04)
//      return;
//      else
//      {

//        vel_now = ( target_point - target_lastpoint ) / ( target_nowtime - target_lasttime ).toSec( );


//        //----------- velocity filter-------------------//
//        target_vel_now=vel_now;
//        target_vel_now=(target_vel_now+target_vel_1+target_vel_2+target_vel_3+target_vel_4)*0.2;
//        target_vel_4=target_vel_3;
//        target_vel_3=target_vel_2;
//        target_vel_2=target_vel_1;
//        target_vel_1=target_vel_now;

//        //----------- state publish  --------------------//
//        //定义一个带有 position velocity yaw多个变量的
//        cotrack::drone_state target_state;
//        target_state.pos_x=target_point[0];
//        target_state.pos_y=target_point[1];
//        target_state.pos_z=target_point[2];
//        target_state.vel_x=target_vel_now[0];
//        target_state.vel_y=target_vel_now[1];
//        target_state.vel_z=target_vel_now[2];
//        target_state.yaw=target_euler[2];
//        target_state.header=msg5->header;
//        drone_4_state_pub.publish(target_state);

//        //----------- state update  -------------------- //
//        target_lastpoint=target_point;
//        target_lasttime= target_nowtime;
//      }
//    }
//}
/*
void StateEstimationNode::relativeposeCb(const geometry_msgs::Pose::ConstPtr posePtr)
{
  relative_pose=posePtr->position;
  target_position[0]=drone1_point[0]+relative_pose.x;
  target_position[1]=drone1_point[1]+relative_pose.y;
  target_position[2]=drone1_point[2]+relative_pose.z;

  if(filter_flag==1)
  {
    //运动模型为匀速运动模型

  }
  if(filter_flag==2)
  {

  }

}
*/
// 四元数转Euler
// q0 q1 q2 q3
// w x y z
//void StateEstimationNode::quaternion_2_euler(Eigen::Quaterniond quat, Eigen::Vector3d &angle)
//{
//    angle(0) = atan2(2.0 * (quat.z() * quat.y() + quat.w() * quat.x()), 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
//    angle(1) = asin(2.0 * (quat.y() * quat.w() - quat.z() * quat.x()));
//  //angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quar.y()), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
//    angle(2) = atan2(2.0 * (quat.z() * quat.w() + quat.x() * quat.y()), 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
//}
void StateEstimationNode::quaternion_2_euler(Eigen::Quaterniond quat, Eigen::Vector3d &angle)
{
    angle(0) = atan2(2.0 * (quat.z() * quat.y() + quat.w() * quat.x()), 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
    angle(1) = asin(2.0 * (quat.y() * quat.w() - quat.z() * quat.x()));
    angle(2) = atan2(2.0 * (quat.z() * quat.w() + quat.x() * quat.y()), -1.0 + 2.0 * (quat.w() * quat.w() + quat.x() * quat.x()));
}
