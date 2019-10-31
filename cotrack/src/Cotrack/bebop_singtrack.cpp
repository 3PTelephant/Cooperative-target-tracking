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

//image
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/aruco/charuco.hpp"


#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"

#include <cotrack/drone_state.h>
using namespace std;
using namespace cv;

//double QuaterniondToEuler(const geometry_msgs::Quaternion quaternion_drone); //四元数转欧拉角
//参数
double pi=3.1415926;
int check_flag;

//-----------------------TOPIC 相关变量----------------
std_msgs::Empty takeoff;
std_msgs::Empty land;
geometry_msgs::Twist bp2_cmd_vel;
cotrack::drone_state bp1_state;
cotrack::drone_state bp2_state;
cotrack::drone_state bp3_state;
cotrack::drone_state bp4_state;
cotrack::drone_state target_state;

//-----------------时间及状态机生成------------
double current_time();
void save_flight_data(std::ofstream& out_file, double timenow);                       //储存数据函数
static ros::Time start_time;                //开始t0 时间戳
bool time_init=false;
double t;
int fly_mode=0;  //1对应  2对应绕圈飞行
int reconfigue_flag=0;

//-----------------目标状态--------------------
double r_target=1;
double w_target=0.05;
double x_t=-0.5+r_target*1;
double y_t=-0.5+r_target*0;
double dot_x_t=-r_target*w_target*0;
double dot_y_t=r_target*w_target*1;
double dot2_x_t=-r_target*w_target*w_target*1;
double dot2_y_t=-r_target*w_target*w_target*0;
double bp1_target_yaw;
double bp2_target_yaw;
double bp3_target_yaw;
double bp4_target_yaw;
//---------------------时变阵型参数------------------
double r;
double w;
double h_x_4;
double h_y_4;
double dot_h_x_4;
double dot_h_y_4;
double dot2_h_x_4;
double dot2_h_y_4;

double h_x_2;
double h_y_2;
double dot_h_x_2;
double dot_h_y_2;
double dot2_h_x_2;
double dot2_h_y_2;

double h_x_1;
double h_y_1;
double dot_h_x_1;
double dot_h_y_1;
double dot2_h_x_1;
double dot2_h_y_1;
//---------------------控制器参数部分----------------

Eigen::Matrix2d Rotation_R2B;
Eigen::Vector2d u_abs;
Eigen::Vector2d u_cooper;
double Kr1 = 0.8;
double Kv1 = 0.5;
double K_coop_r=0.2;
double K_coop_v=0.2;
//---------------------图像变量-------------------
cv::Mat img;

//------------------------回调函数------------------


void vehicle_vrpn(const geometry_msgs::PoseStamped::ConstPtr& msg0)
{
x_t=msg0->pose.position.x;
y_t=msg0->pose.position.y;

}
void bp1_vrpn(const cotrack::drone_state::ConstPtr& msg1)
{
bp1_state=*msg1;
bp1_target_yaw=atan2(-bp1_state.pos_y+y_t,-bp1_state.pos_x+x_t);
cout<<"bebop1 "<<"Pos:"<<bp1_state.pos_x<<","<<bp1_state.pos_y<<endl;
cout<<"bebop1 "<<"Yaw:"<<bp1_state.yaw<<endl;
cout<<"bebop1 "<<"TargetYaw:"<<bp1_target_yaw<<endl;
}
void bp2_vrpn(const cotrack::drone_state::ConstPtr& msg2)
{
bp2_state=*msg2;
bp2_target_yaw=atan2(-bp2_state.pos_y+y_t,-bp2_state.pos_x+x_t);
cout<<"bebop2 "<<"Pos:"<<bp2_state.pos_x<<","<<bp2_state.pos_y<<endl;
cout<<"bebop2 "<<"Yaw:"<<bp2_state.yaw<<endl;
cout<<"bebop2 "<<"TargetYaw:"<<bp2_target_yaw<<endl;
}
void bp3_vrpn(const cotrack::drone_state::ConstPtr& msg3)
{
bp3_state=*msg3;
bp3_target_yaw=atan2(-bp3_state.pos_y+y_t,-bp3_state.pos_x+x_t);
cout<<"bebop3 "<<"Pos:"<<bp3_state.pos_x<<","<<bp3_state.pos_y<<endl;
cout<<"bebop3 "<<"Yaw:"<<bp3_state.yaw<<endl;
cout<<"bebop3 "<<"TargetYaw:"<<bp3_target_yaw<<endl;
}
void bp4_vrpn(const cotrack::drone_state::ConstPtr& msg4)
{
bp4_state=*msg4;
bp4_target_yaw=atan2(-bp4_state.pos_y+y_t,-bp4_state.pos_x+x_t);
cout<<"bebop4 "<<"Pos:"<<bp4_state.pos_x<<","<<bp4_state.pos_y<<endl;
cout<<"bebop4 "<<"Yaw:"<<bp4_state.yaw<<endl;
cout<<"bebop4 "<<"TargetYaw:"<<bp4_target_yaw<<endl;
}
void statemachime_cb(const std_msgs::Int32::ConstPtr& msg5)
{
    fly_mode = msg5->data;
}
void reconfigureflag_cb(const std_msgs::Int32::ConstPtr& msg6)
{
    reconfigue_flag = msg6->data;
}
void target_vrpn(const cotrack::drone_state::ConstPtr& msg7)
{
target_state=*msg7;
x_t=target_state.pos_x;
y_t=target_state.pos_y;
dot_x_t=target_state.vel_y*cos(target_state.yaw);
dot_y_t=target_state.vel_y*sin(target_state.yaw);
dot2_x_t=target_state.vel_y*target_state.vel_x*cos(target_state.yaw+pi/2);
dot2_y_t=target_state.vel_y*target_state.vel_x*sin(target_state.yaw+pi/2);
cout<<"POS:["<<x_t<<","<<y_t<<"]"<<endl;
cout<<"VEL:["<<dot_x_t<<","<<dot_y_t<<"]"<<endl;
cout<<"ACC:["<<dot2_x_t<<","<<dot2_y_t<<"]"<<endl;

}

void  vidCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s",e.what());
    return;
  }
  cv_ptr->image.copyTo(img);

}


int main(int argc, char **argv)
{   ros::init(argc, argv, "bebop_test2");
    ros::NodeHandle nh("~");


    //------------控制部分的topic订阅------------------
    ros::Publisher bp2_takeoff_pub= nh.advertise<std_msgs::Empty>("/bebop2/takeoff", 5);         // 发布 起飞命令
    ros::Publisher bp2_land_pub   = nh.advertise<std_msgs::Empty>("/bebop2/land", 5);            // 发布 降落命令
    ros::Publisher bp2_cmd_pub    = nh.advertise<geometry_msgs::Twist>("/bebop2/cmd_vel", 5);    // 发布 移动命令
    ros::Subscriber statemachime_sub=nh.subscribe<std_msgs::Int32>("/statemachine",10,statemachime_cb);     //飞机状态机模式检测
    ros::Subscriber reconfigureflag_sub=nh.subscribe<std_msgs::Int32>("/reconfigure",10,reconfigureflag_cb);     //飞机状态机模式检测

    //------------订阅小车与飞机的topic-------------------
    ros::Subscriber bp1_state_sub =nh.subscribe<cotrack::drone_state>("/drone_state/drone1",10,bp1_vrpn);
    ros::Subscriber bp2_state_sub =nh.subscribe<cotrack::drone_state>("/drone_state/drone2",10,bp2_vrpn);
    ros::Subscriber bp3_state_sub =nh.subscribe<cotrack::drone_state>("/drone_state/drone3",10,bp3_vrpn);
    ros::Subscriber bp4_state_sub =nh.subscribe<cotrack::drone_state>("/drone_state/drone4",10,bp4_vrpn);
    ros::Subscriber target_state_sub = nh.subscribe<cotrack::drone_state>("/target_state",10,target_vrpn);
    // 订阅图像信息
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber vid_sub=it.subscribe("/bebop2/image_raw",1000,vidCb);

    ros::Rate rate(20);

    //--------------------------存储数据------------------------
    time_t tt = time(NULL);
    tm* time = localtime(&tt);
    char iden_path[256];
    sprintf(iden_path, "/home/jacky/cotrack_ws/data/singletrack-%d-%02d-%02d_%02d-%02d.txt", time->tm_year+1900, time->tm_mon+1, time->tm_mday, time->tm_hour, time->tm_min);
    std::ofstream out_data_file(iden_path);

    if (!out_data_file)
    {
        std::cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " bebop2_x " << " bebop2_y " << " bebop2_vx "<< " bebop2_vy "<< " bebop2_yaw "\
                                << " target_x " << " target_y " << " target_vx "<< " target_vy "<< " target_yaw "\
                                <<std::endl;
    }
      char video_path[256];
      sprintf(video_path, "/home/jacky/cotrack_ws/data/singletrack-%d-%02d-%02d_%02d-%02d.avi", time->tm_year+1900, time->tm_mon+1, time->tm_mday, time->tm_hour, time->tm_min);
      cv::VideoWriter writer(video_path,CV_FOURCC('D','I','V','X'),20,cv::Size(856,480));

    //--------------------------开始起飞并执行命令指令-------------
    cout << "input any key to fly: ";
    cin >> check_flag;


    //------------------------需要补充同时进行飞行任务的指令-------
    t = 0;
    r = 1;
    w =0.2;
    start_time=ros::Time::now();
    while ( ros::ok()&&t<30 )
    {

      switch(fly_mode)
      {
        case 1:
        bp2_takeoff_pub.publish(std_msgs::Empty());   //起飞
        bp2_takeoff_pub.publish(std_msgs::Empty());   //起飞
        cout<<"statemachine="<<fly_mode<<endl;
        fly_mode=0;
        break;

        case 2:
        if(time_init==false)
        {
          start_time=ros::Time::now();
          time_init=true;
        }
        t = current_time();
        cout<<"t:"<<t<<endl;
//        x_t=-0.5+r_target*cos(w_target*t);
//        y_t=-0.5+r_target*sin(w_target*t);
//        dot_x_t=-r_target*w_target*sin(w_target*t);
//        dot_y_t=r_target*w_target*cos(w_target*t);
//        dot2_x_t=-r_target*w_target*w_target*cos(w_target*t);
//        dot2_y_t=-r_target*w_target*w_target*sin(w_target*t);
//        cout<<"x_t,y_t:"<<x_t<<","<<y_t<<endl;
        if(reconfigue_flag==0)
        {
//        h_x_1=r*cos(w*t+3*pi/2);
//        h_y_1=r*sin(w*t+3*pi/2);
//        dot_h_x_1=-r*w*sin(w*t+3*pi/2);
//        dot_h_y_1=r*w*cos(w*t+3*pi/2);
//        dot2_h_x_1=-r*w*w*cos(w*t+3*pi/2);
//        dot2_h_y_1=-r*w*w*sin(w*t+3*pi/2);


        h_x_2=r*cos(w*t+3*pi/2);
        h_y_2=r*sin(w*t+3*pi/2);
        dot_h_x_2=-r*w*sin(w*t+3*pi/2);
        dot_h_y_2=r*w*cos(w*t+3*pi/2);
        dot2_h_x_2=-r*w*w*cos(w*t+3*pi/2);
        dot2_h_y_2=-r*w*w*sin(w*t+3*pi/2);

        cout<<"statemachine="<<fly_mode<<endl;
        ros::spinOnce();
        save_flight_data(out_data_file,t);


        //一号机为右侧的
        u_abs[0]=Kr1*(h_x_2+x_t-bp2_state.pos_x)+Kv1*(dot_h_x_2+dot_x_t-bp2_state.vel_x)+dot2_h_x_2+dot2_x_t;
        u_abs[1]=Kr1*(h_y_2+y_t-bp2_state.pos_y)+Kv1*(dot_h_y_2+dot_y_t-bp2_state.vel_y)+dot2_h_y_2+dot2_y_t;
//        u_cooper[0]=K_coop_r*((bp1_state.pos_x-h_x_1)-(bp2_state.pos_x-h_x_2))+K_coop_v*((bp1_state.vel_x-dot_h_x_1)-(bp2_state.vel_x-dot_h_x_2));
//        u_cooper[1]=K_coop_r*((bp1_state.pos_y-h_y_1)-(bp2_state.pos_y-h_y_2))+K_coop_v*((bp1_state.vel_y-dot_h_y_1)-(bp2_state.vel_y-dot_h_y_2));
        u_cooper[0]=0;
        u_cooper[1]=0;
        u_abs=u_abs+u_cooper;

        Rotation_R2B << cos(bp2_state.yaw), sin(bp2_state.yaw), -sin(bp2_state.yaw), cos(bp2_state.yaw);
        u_abs = Rotation_R2B * u_abs;
        cout<<"u_abs"<<u_abs<<endl;

        }
        else
        {

          h_x_2=r*cos(w*t+4*pi/3);
          h_y_2=r*sin(w*t+4*pi/3);
          dot_h_x_2=-r*w*sin(w*t+4*pi/3);
          dot_h_y_2=r*w*cos(w*t+4*pi/3);
          dot2_h_x_2=-r*w*w*cos(w*t+4*pi/3);
          dot2_h_y_2=-r*w*w*sin(w*t+4*pi/3);

          h_x_4=r*cos(w*t);
          h_y_4=r*sin(w*t);
          dot_h_x_4=-r*w*sin(w*t);
          dot_h_y_4=r*w*cos(w*t);
          dot2_h_x_4=-r*w*w*cos(w*t);
          dot2_h_y_4=-r*w*w*sin(w*t);

          cout<<"statemachine="<<fly_mode<<endl;
          ros::spinOnce();
          save_flight_data(out_data_file,t);

          //一号机为右侧的
          u_abs[0]=Kr1*(h_x_2+x_t-bp2_state.pos_x)+Kv1*(dot_h_x_2+dot_x_t-bp2_state.vel_x)+dot2_h_x_2+dot2_x_t;
          u_abs[1]=Kr1*(h_y_2+y_t-bp2_state.pos_y)+Kv1*(dot_h_y_2+dot_y_t-bp2_state.vel_y)+dot2_h_y_2+dot2_y_t;
//          u_cooper[0]=K_coop_r*((bp4_state.pos_x-h_x_4)-(bp2_state.pos_x-h_x_2))+K_coop_v*((bp4_state.vel_x-dot_h_x_4)-(bp2_state.vel_x-dot_h_x_2));
//          u_cooper[1]=K_coop_r*((bp4_state.pos_y-h_y_4)-(bp2_state.pos_y-h_y_2))+K_coop_v*((bp4_state.vel_y-dot_h_y_4)-(bp2_state.vel_y-dot_h_y_2));
          u_cooper[0]=0;
          u_cooper[1]=0;
          u_abs=u_abs+u_cooper;

          Rotation_R2B << cos(bp2_state.yaw), sin(bp2_state.yaw), -sin(bp2_state.yaw), cos(bp2_state.yaw);
          u_abs = Rotation_R2B * u_abs;
          cout<<"u_abs"<<u_abs<<endl;

        }





        bp2_cmd_vel.linear.x = asin(u_abs[0]/9.8)*9/pi;
        bp2_cmd_vel.linear.y = asin(u_abs[1]/9.8)*9/pi;
        bp2_cmd_vel.linear.z = 0;

        if (bp2_target_yaw-bp2_state.yaw>pi)
        bp2_cmd_vel.angular.z=(bp2_target_yaw-bp2_state.yaw-2*pi)*0.5+1.8*w/pi;
        else if (bp2_target_yaw-bp2_state.yaw<-pi)
            bp2_cmd_vel.angular.z=(bp2_target_yaw-bp2_state.yaw+2*pi)*0.5+1.8*w/pi;
            else
            bp2_cmd_vel.angular.z=(bp2_target_yaw-bp2_state.yaw)*0.5+1.8*w/pi;

        if(abs(bp2_state.pos_x)>2.5||abs(bp2_state.pos_y)>2.5)
        {bp2_cmd_vel.linear.x=0;
          bp2_cmd_vel.linear.y=0;
        }
        bp2_cmd_pub.publish(bp2_cmd_vel);
        writer<<img;

        break;

        default:
        cout<<"statemachine="<<fly_mode<<endl;
        ros::spinOnce();
      }
      rate.sleep();
    }

    //降落
    bp2_land_pub.publish(std_msgs::Empty());
    ros::Duration(3).sleep();
    bp2_land_pub.publish(std_msgs::Empty());
    writer.release();

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

void save_flight_data(std::ofstream& out_file, double timenow)
{
    out_file << timenow <<"  "<<  bp1_state.pos_x <<"  "<< bp1_state.pos_y <<"  "<< bp1_state.vel_x<<"  "<< bp1_state.vel_y <<"  "<< bp1_state.yaw<<"  "\
                              <<  bp2_state.pos_x <<"  "<< bp2_state.pos_y <<"  "<< bp2_state.vel_x<<"  "<< bp2_state.vel_y <<"  "<< bp2_state.yaw<<"  "\
                              <<  bp3_state.pos_x <<"  "<< bp3_state.pos_y <<"  "<< bp3_state.vel_x<<"  "<< bp3_state.vel_y <<"  "<< bp3_state.yaw<<"  "\
                              <<  bp4_state.pos_x <<"  "<< bp4_state.pos_y <<"  "<< bp4_state.vel_x<<"  "<< bp4_state.vel_y <<"  "<< bp4_state.yaw<<"  "\
                              <<  x_t <<"  "<< y_t<<"  "<< dot_x_t<<"  "<< dot_y_t <<"  "<< target_state.yaw<<"  "\
                               <<  std::endl;
}
