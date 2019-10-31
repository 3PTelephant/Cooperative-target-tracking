#include <ros/ros.h>
#include "ControlNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_controlnode");
  ros::NodeHandle nh;
  ControlNode control;
  control.Loop();

}
