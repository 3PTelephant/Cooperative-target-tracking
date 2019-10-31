#pragma once
#ifndef STATEESTIMATION_H
#define STATEESTIMATION_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
//在调用自己声明的msg时需要include内部的包的名字
#include "cotrack/drone_state.h"

#include "std_msgs/String.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"

#include <random>
#include <chrono>
class StateEstimationNode
{
  private:
    ros::NodeHandle nh_;
    //【发布频率】
    int publishFreq;
    //【订阅】视觉的相对位置

    //【订阅】1号机的位姿
    ros::Subscriber drone_1_state_sub;
    //【发布】1号机的状态
    ros::Publisher  drone_1_state_pub;

    //【订阅】2号机的位姿
    ros::Subscriber drone_2_state_sub;
    //【发布】2号机的状态
    ros::Publisher  drone_2_state_pub;
    
    //【订阅】3号机的位姿
    ros::Subscriber drone_3_state_sub;
    //【发布】3号机的状态
    ros::Publisher  drone_3_state_pub;

    //【订阅】4号机的位姿
    ros::Subscriber drone_4_state_sub;
    //【发布】4号机的状态
    ros::Publisher  drone_4_state_pub;

    //【订阅】目标的状态
    ros::Subscriber target_state_sub;
    //【发布】目标的状态
    ros::Publisher target_state_pub;



    //【订阅】相对位置
    ros::Subscriber relative_pose_sub;

    //【回调】1号机位姿
    void drone1stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg1);
    //【回调】2号机位姿
    void drone2stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg2);
    //【回调】3号机位姿
    void drone3stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg3);
    //【回调】4号机位姿
    void drone4stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg4);
    //【回调】目标的位姿
    void targetstateCb(const geometry_msgs::PoseStamped::ConstPtr& msg5);


    //【回调】相对位置
    //void relativeposeCb(const geometry_msgs::Pose::ConstPtr posePtr);


    //【变量】四个飞机的位姿 以及 无人机与小车之间的相对位置
    //【1号机】
    //全部用eigen
    Eigen::Vector3d drone1_point;
    Eigen::Vector3d drone1_lastpoint;
    ros::Time drone1_nowtime;
    ros::Time drone1_lasttime;
    bool drone1_ok;
    Eigen::Quaterniond drone1_q;
    Eigen::Vector3d drone1_euler;
    Eigen::Vector3d drone1_vel_now;
    Eigen::Vector3d drone1_vel_1;
    Eigen::Vector3d drone1_vel_2;
    Eigen::Vector3d drone1_vel_3;
    Eigen::Vector3d drone1_vel_4;

    //【2号机】
    Eigen::Vector3d drone2_point;
    Eigen::Vector3d drone2_lastpoint;
    ros::Time drone2_nowtime;
    ros::Time drone2_lasttime;
    bool drone2_ok;
    Eigen::Quaterniond drone2_q;
    Eigen::Vector3d drone2_euler;
    Eigen::Vector3d drone2_vel_now;
    Eigen::Vector3d drone2_vel_1;
    Eigen::Vector3d drone2_vel_2;
    Eigen::Vector3d drone2_vel_3;
    Eigen::Vector3d drone2_vel_4;
    //【3号机】
     Eigen::Vector3d drone3_point;
    Eigen::Vector3d drone3_lastpoint;
    ros::Time drone3_nowtime;
    ros::Time drone3_lasttime;
    bool drone3_ok;
    Eigen::Quaterniond drone3_q;
    Eigen::Vector3d drone3_euler;
    Eigen::Vector3d drone3_vel_now;
    Eigen::Vector3d drone3_vel_1;
    Eigen::Vector3d drone3_vel_2;
    Eigen::Vector3d drone3_vel_3;
    Eigen::Vector3d drone3_vel_4;
    //【4号机】
    Eigen::Vector3d drone4_point;
    Eigen::Vector3d drone4_lastpoint;
    ros::Time drone4_nowtime;
    ros::Time drone4_lasttime;
    bool drone4_ok;
    Eigen::Quaterniond drone4_q;
    Eigen::Vector3d drone4_euler;
    Eigen::Vector3d drone4_vel_now;
    Eigen::Vector3d drone4_vel_1;
    Eigen::Vector3d drone4_vel_2;
    Eigen::Vector3d drone4_vel_3;
    Eigen::Vector3d drone4_vel_4;

    //【目标状态变量】
    Eigen::Vector3d target_point;
    Eigen::Vector3d target_lastpoint;
    Eigen::Vector3d vel_now;
    ros::Time target_nowtime;
    ros::Time target_lasttime;

    bool target_ok;
    Eigen::Quaterniond target_q;
    Eigen::Vector3d target_euler;
    Eigen::Vector3d target_lasteuler;
    int n_2pi;

    double target_vel_linear;
    double target_vel_angular;
    bool target_state_init;

    Eigen::Vector3d target_vel_now;
    geometry_msgs::Point relative_pose;


    //【滤波算法选择变量】
    /*说明
     * 1-->卡尔曼滤波(两种计算目标位置均可以)  2-->UKF(假定相机夹角为30度即云台自稳模式)
     * 3-->PF粒子滤波       4.EKF待做
     */
    int filter_flag;

    Eigen::Matrix<double,5,5> Jacb;
    Eigen::Matrix<double,3,5> H;
    Eigen::Matrix<double,5,3> K;
    Eigen::Matrix<double,5,5> P;
    Eigen::Matrix<double,5,5> Q;
    Eigen::Matrix<double,3,3> R;
    Eigen::Matrix<double,5,1> target_state;
    Eigen::Matrix<double,5,1> prect_state;
    Eigen::Matrix<double,5,1> correct_state;
    Eigen::Matrix<double,3,1> observe_state;
    Eigen::Matrix<double,3,1> prect_part_state;
    double pi;
    //四元数转欧拉角
    void quaternion_2_euler(Eigen::Quaterniond quat, Eigen::Vector3d &angle);

    //
  public:
     StateEstimationNode();
     ~StateEstimationNode();


     //【滤波算法主循环】
     void Loop();

};
#endif
