#include <ros/ros.h>
#include "TargetDetection.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_targetdetection");
  ros::NodeHandle nh_;
  TargetDetectionNode detector;
  detector.Loop();
}
