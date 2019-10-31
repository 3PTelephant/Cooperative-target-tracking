#pragma once
#ifndef TARGETDETECTION_H
#define TARGETDETECTION_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"

//#include "msg_test/filter_state.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

class TargetDetectionNode
{
private:
  ros::NodeHandle nh_;
  //【发布频率】
  int publishFreq;
  //【订阅】image_transport用来进行图像数据的传输
  image_transport::Subscriber vid_sub;
  //【发布】无人机和小车相对位姿以及flag
  ros::Publisher pose_pub;
  //【订阅】1号机的位姿
  ros::Subscriber drone_1_state_sub;
  //【发布】1号机的状态
  ros::Publisher  target_state_pub;
  //【回调】1号机位姿
  void drone1stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg1);

  //【变量】1号无人机的位置
  Eigen::Vector3d drone1_point;
  Eigen::Quaterniond drone1_q;
  Eigen::Vector3d drone1_euler;

  //【发布】无人机和小车相对偏航角
  //ros::Publisher yaw_pub;
  //【发布】是否识别到目标标志位
  //ros::Publisher position_flag_pub;
  //【变量】检测成功标志位----Point.orientation.w=1为检测成功 =0为检测失败
  //geometry_msgs::Pose flag_position;
  bool solve_flag;

  //drone is World coordinate
  Eigen::Vector3f last_target_position,current_target_position;

  //【变量】旋转向量和平移向量
  cv::Mat rvec,tvec;
  //【变量】相机内参及畸变系数
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients; // 
  void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
  void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz);
  void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz);
public:
  TargetDetectionNode();
  ~TargetDetectionNode();

  //【回调】ROS message callbacks
  void vidCb(const sensor_msgs::ImageConstPtr& msg);

  //【目标识别主循环】
  void Loop();

  //set camera parameters
  void setCameraMatrix(double fx,double fy,double u0,double v0);
  void setDistortionCoefficients(double k_1,double k_2,double p_1,double p_2,double k_3);

};

#endif
