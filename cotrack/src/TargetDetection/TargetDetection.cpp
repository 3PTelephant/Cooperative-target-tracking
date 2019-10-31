#include "TargetDetection.h"

using namespace std;
using namespace cv;


void TargetDetectionNode::drone1stateCb(const geometry_msgs::PoseStamped::ConstPtr& msg1)
{
  drone1_point[0]=msg1->pose.position.x;
  drone1_point[1]=msg1->pose.position.y;
  drone1_point[2]=msg1->pose.position.z;
}

TargetDetectionNode::TargetDetectionNode()
{
  //声明
  image_transport::ImageTransport it(nh_);
  //发布频率
  publishFreq=20;
  //【订阅】bebop传回来的图像数据
  vid_sub = it.subscribe("/bebop/image_raw",1000,&TargetDetectionNode::vidCb,this);
  //【订阅】当前无人机位置
  drone_1_state_sub=nh_.subscribe("/vrpn_client_node/drone1/pose",2,&TargetDetectionNode::drone1stateCb,this);
  //【发布】目标当前状态 可以对比两种解算方式
  //(TBD)订阅小车位置  简单的就是读取小车的偏航角将marker系同世界系进行旋转对齐
  //(TBD)利用无人机的自身信息和解算的偏航角 得到小车的偏航角对齐marker系与世界系
  //(TBD)直接利用飞机的位姿和相机的夹角 将相机坐标系下的位置旋转到世界系去 即 Camera->Body->World
  target_state_pub=nh_.advertise<geometry_msgs::Pose>("/target_state",10);
  solve_flag=1;


  //【发布】bebop与目标之间的相对位置与偏航角  带有flag
  /*   -------note-------------
   * Point         -----relative point
  //orientation.z -----yaw
  //orientation.w -----flag
  */
  pose_pub=nh_.advertise<geometry_msgs::Pose>("/relative_pose/drone1",10);

  //【发布】bebop与目标之间的相对偏航角
  //yaw_pub=nh_.advertise<geometry_msgs::Pose>("/relative_yaw",10);
  //【发布】目标识别成功的标志位
  //position_flag_pub=nh_.advertise<geometry_msgs::Pose>("/detected_flag/drone1",10);


  //相机内部参数
  float fx,fy,x_0,y_0;
  //相机畸变系数
  float k1,k2,p1,p2,k3;

  //读取参数文档camera_param.yaml中的参数值；
  nh_.param<float>("fx", fx, 547.683266);
  nh_.param<float>("fy", fy, 537.261402);
  nh_.param<float>("x0", x_0, 428.718662);
  nh_.param<float>("y0", y_0, 230.572967);

  nh_.param<float>("k1", k1, -0.008643);
  nh_.param<float>("k2", k2, 0.008943);
  nh_.param<float>("p1", p1, -0.005879);
  nh_.param<float>("p2", p2, 0.001494);
  nh_.param<float>("k3", k3, 0.0);



  //设置相机内参及畸变系数
  setCameraMatrix(fx,fy,x_0,y_0); //wait for adding! 提取到config中
  setDistortionCoefficients(k1,k2,p1,p2,k3);

}
TargetDetectionNode::~TargetDetectionNode()
{

}

void  TargetDetectionNode::vidCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img;
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

  //ArUco 配置及检测
  Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(10);
  std::vector<int> markerids;
  vector<vector<Point2f> > markerCorners,rejectedCandidate;
  Ptr<cv::aruco::DetectorParameters> parameters=cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(img,dictionary,markerCorners,markerids,parameters,rejectedCandidate);
  //cluster算法
  //-------------------多于一个目标被识别到，进入算法-----------------
  if (markerids.size()>0)
  {
      //未处理后的位置
      vector<cv::Point3f> vec_Position_OcInW;
      vector<double> vec_yaw;
      cv::Point3f A1_Sum_Position_OcInW(0,0,0);
      double A1_Sum_yaw=0.0;
      for(int t=0;t<markerids.size();t++)
      {
          vector<double> rv(3),tv(3);
          cv::Mat rvec(rv),tvec(tv);
          cv::Mat RoteM, TransM;
          //C2W代表 相机坐标系转换到世界坐标系  W2C代表 世界坐标系转换到相机坐标系 Theta为欧拉角
          cv::Point3f Theta_C2W;
          cv::Point3f Theta_W2C;
          cv::Point3f Position_OcInW;


          //--------------对每一个Marker的相对位置进行解算----------------
          vector<vector<Point2f> > singMarkerCorner_10, singMarkerCorner_15;
          if (markerids[t]==1||markerids[t]==3||markerids[t]==7||markerids[t]==9)
          {
            singMarkerCorner_15.push_back(markerCorners[t]);
            cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_15,0.15,camera_matrix,distortion_coefficients,rvec,tvec);

          }
          else
          {
             singMarkerCorner_10.push_back(markerCorners[t]);
             cv::aruco::estimatePoseSingleMarkers(singMarkerCorner_10,0.10,camera_matrix,distortion_coefficients,rvec,tvec);

          }

          //将解算的位置转化成旋转矩阵 并旋转计算无人机相对于目标的位置
          double rm[9];
          RoteM = cv::Mat(3, 3, CV_64FC1, rm);
          //利用罗德里格斯公式将旋转向量转成旋转矩阵
          Rodrigues(rvec, RoteM);
          double r11 = RoteM.ptr<double>(0)[0];
          double r12 = RoteM.ptr<double>(0)[1];
          double r13 = RoteM.ptr<double>(0)[2];
          double r21 = RoteM.ptr<double>(1)[0];
          double r22 = RoteM.ptr<double>(1)[1];
          double r23 = RoteM.ptr<double>(1)[2];
          double r31 = RoteM.ptr<double>(2)[0];
          double r32 = RoteM.ptr<double>(2)[1];
          double r33 = RoteM.ptr<double>(2)[2];
          TransM = tvec;
          //计算欧拉角
          double thetaz = atan2(r21, r11) / CV_PI * 180;
          double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
          double thetax = atan2(r32, r33) / CV_PI * 180;

          Theta_C2W.z = thetaz;
          Theta_C2W.y = thetay;
          Theta_C2W.x = thetax;

          Theta_W2C.x = -1 * thetax;
          Theta_W2C.y = -1 * thetay;
          Theta_W2C.z = -1 * thetaz;
          //偏移向量
          double tx = tvec.ptr<double>(0)[0];
          double ty = tvec.ptr<double>(0)[1];
          double tz = tvec.ptr<double>(0)[2];
          double x = tx, y = ty, z = tz;

          //进行三次旋转得到相机光心在世界坐标系的位置
          CodeRotateByZ(x, y, -1 * thetaz, x, y);
          CodeRotateByY(x, z, -1 * thetay, x, z);
          CodeRotateByX(y, z, -1 * thetax, y, z);
          Position_OcInW.x = x*-1;
          Position_OcInW.y = y*-1;
          Position_OcInW.z = z*-1;

          //计算偏航角之差
          Eigen::Matrix3d rotateMatrix;
          rotateMatrix<<r11,r12,r13,r21,r22,r23,r31,r32,r33;
          Eigen::Vector3d eulerVec;
          eulerVec(0)=(Theta_C2W.z+90)/180*CV_PI;
          vec_yaw.push_back(eulerVec(0));

          //根据Marker ID对相对位置进行偏移
          switch (markerids[t])
          {
              case 1:
              {
                  Position_OcInW.x-=0.175;
                  Position_OcInW.y+=0.175;
                  break;
              }
              case 2:
              {
                  Position_OcInW.y+=0.2;
                  break;
              }
              case 3:
              {
                  Position_OcInW.x+=0.175;
                  Position_OcInW.y+=0.175;
                  break;
              }
              case 4:
              {
                  Position_OcInW.x-=0.2;
                  break;
              }
              case 5:
                  break;
              case 6:
              {
                  Position_OcInW.x+=0.2;
                  break;
              }
              case 7:
              {
                  Position_OcInW.x-=0.175;
                  Position_OcInW.y-=0.175;
                  break;
              }
              case 8:
              {
                  Position_OcInW.y-=0.2;
                  break;
              }
              case 9:
              {
                  Position_OcInW.x+=0.175;
                  Position_OcInW.y-=0.175;
                  break;
              }
          }
          //------------switch结束------------
          vec_Position_OcInW.push_back(Position_OcInW);

          A1_Sum_Position_OcInW+=Position_OcInW;
          A1_Sum_yaw+=eulerVec(0); //待修改


      }
      //解算位置的平均值
      cv::Point3f A1_Position_OcInW(0,0,0);
      double A1_yaw=0.0;
      int marker_count=markerids.size();
      A1_Position_OcInW=A1_Sum_Position_OcInW/marker_count;
      A1_yaw=A1_Sum_yaw/marker_count;


      //将解算后的位置发给控制端
      geometry_msgs::Pose pose_msg;
      pose_msg.position.x=A1_Position_OcInW.x;
      pose_msg.position.y=A1_Position_OcInW.y;
      pose_msg.position.z=A1_Position_OcInW.z;
      pose_msg.orientation.z=A1_yaw;
      pose_msg.orientation.w=1;
      pose_pub.publish(pose_msg);


      //将解算的目标位置发送出来
      geometry_msgs::Pose target_msg;
      //【方法1】利用无人机在marker系下的位置
      if (solve_flag==1)
      {
        //理论部分使用坐标系转来转去
        //实际的获取小车的偏航角来（optitrack）或者（marker解出来的yaw+无人机自身的yaw）
        /*
           矫正后观测的小车的位置
           double yaw_vehicle=drone1_euler[2]+A1_yaw;
           target_msg.position.x=drone1_point[0]+A1_Position_OcInW.x*cos(yaw_vehicle)+A1_Position_OcInW.y*sin(yaw_vehicle);
           target_msg.position.y=drone1_point[1]+A1_Position_OcInW.x*-sin(yaw_vehicle)+A1_Position_OcInW.y*cos(yaw_vehicle);
         */
        target_msg.position.x=drone1_point[0]+A1_Position_OcInW.x;
        target_msg.position.y=drone1_point[1]+A1_Position_OcInW.y;
        //是否需要从高度环节进行矫正
      }

      //【方法2】利用marker系在相机坐标系的位置
      if (solve_flag==2)
      {
      //（TBD）
      }



      target_msg.orientation.w=1;
      target_state_pub.publish(target_msg);

  }
  else
  {
      geometry_msgs::Pose pose_msg;
      pose_msg.orientation.w=0;
      pose_pub.publish(pose_msg);

      //不考虑解算的flag
      geometry_msgs::Pose target_msg;
      if (solve_flag==1)
      {
        target_msg.orientation.w=0;
      }
      if (solve_flag==2)
      {

      }
      target_state_pub.publish(target_msg);

  }
  cv::imshow("TEST",img);
  cvWaitKey(1);
}
void TargetDetectionNode::Loop()
{
  ros::Rate pub_rate(publishFreq);
  while (nh_.ok())
  {

     ros::spinOnce();

     pub_rate.sleep();
  }

}

void TargetDetectionNode::setCameraMatrix(double fx, double fy, double u0, double v0)
{
  camera_matrix=cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
  camera_matrix.ptr<double>(0)[0]=fx;
  camera_matrix.ptr<double>(0)[2]=u0;
  camera_matrix.ptr<double>(1)[1]=fy;
  camera_matrix.ptr<double>(1)[2]=v0;
  camera_matrix.ptr<double>(2)[2]=1.0f;
}
void TargetDetectionNode::setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3)
{
  distortion_coefficients=cv::Mat(5,1,CV_64FC1,cv::Scalar::all(0));
  distortion_coefficients.ptr<double>(0)[0]=k_1;
  distortion_coefficients.ptr<double>(1)[0]=k_2;
  distortion_coefficients.ptr<double>(2)[0]=p_1;
  distortion_coefficients.ptr<double>(3)[0]=p_2;
  distortion_coefficients.ptr<double>(4)[0]=k_3;
}
//-----------------利用Euler角进行三次旋转得到无人机相对目标的位置------------------
void TargetDetectionNode::CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}
void TargetDetectionNode::CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * CV_PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}
void TargetDetectionNode::CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}
