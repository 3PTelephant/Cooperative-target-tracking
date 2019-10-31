#ifndef KALMANFILTERNODE_H
#define KALMANFILTERNODE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"

class KalmanFilterNode
{

private:
  ros::NodeHandle nh_;
  //发布频率
  int publishFreq;
  //更新步长
  double delta_t;
  //系统状态变量
  Eigen::VectorXd target_state;
  //观测值参数
  Eigen::VectorXd target_state_meas;
  //系统初始化
  bool targetstate_is_init;
  bool targetstate_is_meas;
  bool target_found;
  bool X_evlt_is_assign;

  //【订阅】目标的状态
  ros::Subscriber target_state_sub;
  //【发布】
  //【回调】目标的状态
  void targetstateCb(const geometry_msgs::Pose::ConstPtr msg);


public:
  KalmanFilterNode();
  ~KalmanFilterNode();


  void Loop();
};

#endif // KALMANFILTER_H
