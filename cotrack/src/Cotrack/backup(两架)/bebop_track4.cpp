using namespace std;
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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


//topic
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

#include <cotrack/drone_state.h>

//double QuaterniondToEuler(const geometry_msgs::Quaternion quaternion_drone); //四元数转欧拉角
//参数
double pi=3.1415926;
int check_flag;

//-----------------------TOPIC 相关变量----------------
std_msgs::Empty takeoff;
std_msgs::Empty land;
geometry_msgs::Twist bp4_cmd_vel;
cotrack::drone_state bp4_state;
cotrack::drone_state bp3_state;

//-----------------时间及状态机生成------------
double current_time();
static ros::Time start_time;                //开始t0 时间戳
bool time_init=false;
double t;
int fly_mode=1;  //1对应  2对应绕圈飞行

//-----------------目标状态--------------------
double x_t=0;
double y_t=0;
double dot_x_t=0;
double dot_y_t=0;
double dot2_x_t=0;
double dot2_y_t=0;
double bp4_target_yaw;
double bp3_target_yaw;
//---------------------时变阵型参数------------------
double r;
double w;
double h_x_4;
double h_y_4;
double dot_h_x_4;
double dot_h_y_4;
double dot2_h_x_4;
double dot2_h_y_4;

double h_x_3;
double h_y_3;
double dot_h_x_3;
double dot_h_y_3;
double dot2_h_x_3;
double dot2_h_y_3;
//---------------------控制器参数部分----------------

Eigen::Matrix2d Rotation_R2B;
Eigen::Vector2d u_abs;
Eigen::Vector2d u_cooper;
double Kr1 = 0.8;
double Kv1 = 0.5;
double K_coop_r=0.5;
double K_coop_v=0.2;


//------------------------回调函数------------------


void vehicle_vrpn(const geometry_msgs::PoseStamped::ConstPtr& msg1)
{
x_t=msg1->pose.position.x;
y_t=msg1->pose.position.y;

}
void bp4_vrpn(const cotrack::drone_state::ConstPtr& msg)
{
bp4_state=*msg;
bp4_target_yaw=atan2(-bp4_state.pos_y+y_t,-bp4_state.pos_x+x_t);
cout<<"bebop4 "<<"Pos:"<<bp4_state.pos_x<<","<<bp4_state.pos_y<<endl;
cout<<"bebop4 "<<"Yaw:"<<bp4_state.yaw<<endl;
cout<<"bebop4 "<<"TargetYaw:"<<bp4_target_yaw<<endl;
}

void bp3_vrpn(const cotrack::drone_state::ConstPtr& msg3)
{
bp3_state=*msg3;
bp3_target_yaw=atan2(-bp3_state.pos_y+y_t,-bp3_state.pos_x+x_t);
cout<<"bebop3 "<<"Pos:"<<bp3_state.pos_x<<","<<bp3_state.pos_y<<endl;
cout<<"bebop3 "<<"Yaw:"<<bp3_state.yaw<<endl;
cout<<"bebop3 "<<"TargetYaw:"<<bp3_target_yaw<<endl;
}
void statemachime_cb(const std_msgs::Int32::ConstPtr& msg2)
{
    fly_mode = msg2->data;
}





int main(int argc, char **argv)
{   ros::init(argc, argv, "bebop_test4");
    ros::NodeHandle nh("~");


    //------------控制部分的topic订阅------------------
    ros::Publisher bp4_takeoff_pub= nh.advertise<std_msgs::Empty>("/bebop4/takeoff", 5);         // 发布 起飞命令
    ros::Publisher bp4_land_pub   = nh.advertise<std_msgs::Empty>("/bebop4/land", 5);            // 发布 降落命令
    ros::Publisher bp4_cmd_pub    = nh.advertise<geometry_msgs::Twist>("/bebop4/cmd_vel", 5);    // 发布 移动命令
    ros::Subscriber statemachime_sub=nh.subscribe<std_msgs::Int32>("/statemachine",10,statemachime_cb);     //飞机状态机模式检测

    //------------订阅小车与飞机的topic-------------------
    ros::Subscriber bp4_state_sub =nh.subscribe<cotrack::drone_state>("/drone_state/drone4",10,bp4_vrpn);


    ros::Rate rate(20);

    cout<<"x_t"<<x_t<<endl;
    //--------------------------开始起飞并执行命令指令-------------
    cout << "input any key to fly: ";
    cin >> check_flag;
    ros::Duration(3).sleep();
    bp4_takeoff_pub.publish(std_msgs::Empty());   //起飞
    ros::Duration(3).sleep();

    //------------------------需要补充同时进行飞行任务的指令-------
    t = 0;
    r = 0.8;
    w =0.2;
    start_time=ros::Time::now();
    while ( ros::ok()&&t<30 )
    {


      if(fly_mode==2)
      {
        if(time_init==false)
        {
          start_time=ros::Time::now();
          time_init=true;
        }
        t = current_time();
        cout<<"t:"<<t<<endl;
        h_x_4=r*cos(w*t);
        h_y_4=r*sin(w*t);
        dot_h_x_4=-r*w*sin(w*t);
        dot_h_y_4=r*w*cos(w*t);
        dot2_h_x_4=-r*w*w*cos(w*t);
        dot2_h_y_4=-r*w*w*sin(w*t);

        h_x_3=r*cos(w*t+pi);
        h_y_3=r*sin(w*t+pi);
        dot_h_x_3=-r*w*sin(w*t+pi);
        dot_h_y_3=r*w*cos(w*t+pi);
        dot2_h_x_3=-r*w*w*cos(w*t+pi);
        dot2_h_y_3=-r*w*w*sin(w*t+pi);

        cout<<"statemachine="<<fly_mode<<endl;
        ros::spinOnce();
        //一号机为右侧的
        u_abs[0]=Kr1*(h_x_4+x_t-bp4_state.pos_x)+Kv1*(dot_h_x_4-bp4_state.vel_x)+dot2_h_x_4;
        u_abs[1]=Kr1*(h_y_4+y_t-bp4_state.pos_y)+Kv1*(dot_h_y_4-bp4_state.vel_y)+dot2_h_y_4;

        u_cooper[0]=K_coop_r*((bp3_state.pos_x-h_x_3)-(bp4_state.pos_x-h_x_4))+K_coop_v*((bp3_state.vel_x-dot_h_x_3)-(bp4_state.vel_x-dot_h_x_4));
        u_cooper[1]=K_coop_r*((bp3_state.pos_y-h_y_3)-(bp4_state.pos_y-h_y_4))+K_coop_v*((bp3_state.vel_y-dot_h_y_3)-(bp4_state.vel_y-dot_h_y_4));
//        u_cooper[0]=0;
//        u_cooper[1]=0;
        cout<<"U_cooper"<<u_cooper[0]<<endl;
        u_abs=u_abs+u_cooper;

        Rotation_R2B << cos(bp4_state.yaw), sin(bp4_state.yaw), -sin(bp4_state.yaw), cos(bp4_state.yaw);
        u_abs = Rotation_R2B * u_abs;
        cout<<"u_abs"<<u_abs<<endl;
        bp4_cmd_vel.linear.x = asin(u_abs[0]/9.8)*9/pi;
        bp4_cmd_vel.linear.y = asin(u_abs[1]/9.8)*9/pi;
        bp4_cmd_vel.linear.z = 0;

        if (bp4_target_yaw-bp4_state.yaw>pi)
        bp4_cmd_vel.angular.z=(bp4_target_yaw-bp4_state.yaw-2*pi)*0.5+1.8*w/pi;
        else if (bp4_target_yaw-bp4_state.yaw<-pi)
            bp4_cmd_vel.angular.z=(bp4_target_yaw-bp4_state.yaw+2*pi)*0.5+1.8*w/pi;
            else
            bp4_cmd_vel.angular.z=(bp4_target_yaw-bp4_state.yaw)*0.5+1.8*w/pi;

        if(abs(bp4_state.pos_x)>2.5||abs(bp4_state.pos_y)>2.5)
        {bp4_cmd_vel.linear.x=0;
          bp4_cmd_vel.linear.y=0;
        }
        bp4_cmd_pub.publish(bp4_cmd_vel);
      }
      else
      {
        ros::spinOnce();

      }
      //if (bp4_target_yaw-bp4_current_yaw>pi)
      //bp4_cmd_vel.angular.z=(bp4_target_yaw-bp4_current_yaw-2*pi)*0.2+w/pi;
      //else if (bp4_target_yaw-bp4_current_yaw<-pi)
      //bp4_cmd_vel.angular.z=(bp4_target_yaw-bp4_current_yaw+2*pi)*0.2+w/pi;
      //else
      //bp4_cmd_vel.angular.z=(bp4_target_yaw-bp4_current_yaw)*0.2+w/pi;

      rate.sleep();


      }

       //降落
          bp4_land_pub.publish(std_msgs::Empty());
          ros::Duration(3).sleep();
          bp4_land_pub.publish(std_msgs::Empty());

}

double QuaterniondToEuler(const geometry_msgs::Quaternion quaternion_drone)
{   Eigen::Quaterniond q;  //四元数

    float quat[4];
    float angle[3];
    quat[0] = quaternion_drone.w;
    quat[1] = quaternion_drone.x;
    quat[2] = quaternion_drone.y;
    quat[3] = quaternion_drone.z;

    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));

    return angle[2];
}

double current_time()
{ros::Duration elapsed_time = ros::Time::now() - start_time;
 double secs =elapsed_time.toSec();
 return secs;
}
