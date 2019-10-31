#include <ros/ros.h>
#include "StateEstimation.h"

/*
 * 计算每个飞机的速度，并与位置绑定重新发布
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_stateestimation");
  ros::NodeHandle nh_;
  StateEstimationNode estimator;
  estimator.Loop();
}
