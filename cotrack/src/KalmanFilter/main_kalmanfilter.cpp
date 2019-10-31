#include <ros/ros.h>
#include "KalmanFilter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_kalmanfilter");
  ros::NodeHandle nh_;

  KalmanFilterNode filter;
  filter.Loop();
}
