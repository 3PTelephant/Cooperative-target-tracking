#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "time.h"
#include "fstream"
#include "iostream"
#include "math.h"

#include "RPnP.h"


using namespace std;
using namespace cv;

//解算出的所有位姿都进行保存
//marker1: 旋转向量1x3，平移向量1x3



double threshold_error=0.4;
//---------------------------variables---------------------------------------
//------------ROS TOPIC---------
//【订阅】无人机位置
ros::Subscriber drone_pose_sub;
//【订阅】小车位置
ros::Subscriber vehicle_pose_sub;
//【发布】无人机和小车相对位置
ros::Publisher position_pub;
//【发布】无人机和小车相对偏航角
ros::Publisher yaw_pub;
//【发布】是否识别到目标标志位
ros::Publisher position_flag_pub;

//-------------VISION-----------
Mat img;



//-------------TIME-------------
ros::Time begin_time;
float cur_time;
float photo_time;


//是否检测到标志位-----Point.orientation.w=1为检测成功 =0为检测失败
geometry_msgs::Pose flag_position;
//无人机位姿message
geometry_msgs::Pose pos_drone_optitrack;
Eigen::Vector3d euler_drone_optitrack;
Eigen::Quaterniond q_drone;
//小车位姿message
geometry_msgs::Pose pos_vehicle_optitrack;
Eigen::Vector3d euler_vehicle_optitrack;
Eigen::Quaterniond q_vehicle;

//保存的上次观测的位置 用于cluster算法使用
Eigen::Vector3d last_position;
bool bool_last_position=false;

//-----------------利用Euler角进行三次旋转得到无人机相对目标的位置------------------
void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}
void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * CV_PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}
void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}
// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(Eigen::Quaterniond quat, Eigen::Vector3d &angle)
{
    angle(0) = atan2(2.0 * (quat.z() * quat.y() + quat.w() * quat.x()), 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
    angle(1) = asin(2.0 * (quat.y() * quat.w() - quat.z() * quat.x()));
  //angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    angle(2) = atan2(2.0 * (quat.z() * quat.w() + quat.x() * quat.y()), 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
}

//获取系统时间
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimate2");
    ros::NodeHandle nh("~");

    //--------------------------相机参数赋值---------------------
    //相机内参
    Mat camera_matrix;
    //相机畸变参数k1 k2 p1 p2 k3
    Mat distortion_coefficients;
    camera_matrix =cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0]=580;
    camera_matrix.ptr<double>(0)[2]=344.760522;
    camera_matrix.ptr<double>(1)[1]=580;
    camera_matrix.ptr<double>(1)[2]=256.145560;
    camera_matrix.ptr<double>(2)[2]=1.0f;

    distortion_coefficients=cv::Mat(5,1,CV_64FC1,cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0]=-0.409105;
    distortion_coefficients.ptr<double>(1)[0]=0.200162;
    distortion_coefficients.ptr<double>(2)[0]=0.003778;
    distortion_coefficients.ptr<double>(3)[0]=0.000588;
    distortion_coefficients.ptr<double>(4)[0]=0.0;

    //ArUco Marker字典选择以及旋转向量和评议向量初始化
    Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(10);



    time_t tt = time(NULL);
    tm* t = localtime(&tt);
    //-------------------------数据储存初始化--------------------
    char data_path[256];
    sprintf(data_path,"/home/jacky/cotrack_ws/data/RPNPTEST-%d-%02d-%02d_%02d-%02d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min);
    std::ofstream out_data_file(data_path);
    if (!out_data_file)
    {
        std::cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " point1.x " << " point1.y " \
                                << " point2.x " << " point2.y " \
                                << " point3.x " << " point3.y " \
                                << " point4.x " << " point4.y " \
                                <<std::endl;
    }
    begin_time = ros::Time::now();

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate loopRate(20);
    VideoCapture capture;
    capture.open("/home/jacky/cotrack_ws/333.avi");

    //----------------------------------------主循环------------------------------------
    while(capture.isOpened()&&ros::ok())
    {
        clock_t start=clock();
        capture>>img;
        ros::spinOnce();


        //------------------调用ArUco Marker库对图像进行识别--------------
        //markerids存储每个识别到二维码的编号  markerCorners每个二维码对应的四个角点的像素坐标
        std::vector<int> markerids;
        vector<vector<Point2f> > markerCorners,rejectedCandidate;
        Ptr<cv::aruco::DetectorParameters> parameters=cv::aruco::DetectorParameters::create();
        parameters->cornerRefinementMethod=true;
        cv::aruco::detectMarkers(img,dictionary,markerCorners,markerids,parameters,rejectedCandidate);
//        cout<<"point1"<<markerCorners[0][0].x<<endl;
//        cout<<"point2"<<markerCorners[0][1].x<<endl;
//        cout<<"point3"<<markerCorners[0][2].x<<endl;
//        cout<<"point4"<<markerCorners[0][3].x<<endl;

        cur_time = get_dt(begin_time);
        photo_time =cur_time;
        out_data_file << cur_time <<"  "<< markerCorners[0][0].x <<"  "<< markerCorners[0][0].y <<"  "\
                                        << markerCorners[0][1].x <<"  "<< markerCorners[0][1].y <<"  "\
                                        << markerCorners[0][2].x <<"  "<< markerCorners[0][2].y <<"  "\
                                        << markerCorners[0][3].x <<"  "<< markerCorners[0][3].y <<"  "\

                                        << std::endl;
        vector<double> rv(3),tv(3);
        cv::Mat rvec(rv),tvec(tv);
        //每次循环重置所有变量



        //-------------------多于一个目标被识别到，进入算法-----------------
        if (markerids.size()>0)
        {


                cv::Mat RoteM, TransM;
                //C2W代表 相机坐标系转换到世界坐标系  W2C代表 世界坐标系转换到相机坐标系 Theta为欧拉角
                cv::Point3f Theta_C2W;
                cv::Point3f Theta_W2C;
                cv::Point3f Position_OcInW;



                cv::aruco::estimatePoseSingleMarkers(markerCorners,0.20,camera_matrix,distortion_coefficients,rvec,tvec);

                //将解算的位置转化成旋转矩阵 并旋转计算无人机相对于目标的位置
                double rm[9];
                RoteM = cv::Mat(3, 3, CV_64FC1, rm);
                cout<<"R"<<rvec<<"t"<<tvec<<endl;
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

                Eigen::Matrix3d rotateMatrix;
                rotateMatrix<<r11,r12,r13,r21,r22,r23,r31,r32,r33;




                std::vector<Point3d> WorldPts;
                std::vector<Point2d> Imgpts;
                Mat_<double> ALLrotM=Mat(3,3,CV_64F);
                Mat_<double> ALLTransM=Mat(3,1,CV_64F);

                WorldPts.clear();
                WorldPts.push_back(Point3d(-100.0f,100.0f,0.0f));//World size is described by mm
                WorldPts.push_back(Point3d(100.0f,100.0f,0.0f));
                WorldPts.push_back(Point3d(100.0f,-100.0f,0.0f));
                WorldPts.push_back(Point3d(-100.0f,-100.0f,0.0f));
                cout<<"world ok "<<endl;
                vector<Point2d> corner;

                undistort(markerCorners[0],corner,camera_matrix,distortion_coefficients);

                Imgpts.clear();
                Imgpts.push_back(corner[0]);
                Imgpts.push_back(corner[1]);
                Imgpts.push_back(corner[2]);
                Imgpts.push_back(corner[3]);
                cout<<"world ok "<<endl;

                Point2d PrincipalPoint;
                double focallength=580;

                PrincipalPoint.x=344.760522;
                PrincipalPoint.y=256.145560;
                if(RPnP(WorldPts,Imgpts,ALLrotM,ALLTransM,PrincipalPoint,focallength))
              {
                Mat tmpr;
                Rodrigues(ALLrotM,tmpr);
                LookMat(tmpr,"Gotten R");
                LookMat(ALLTransM,"Gotten T");
              }
                else
                    printf("The pose can not be calculated by RPnP");
        }
        else
        {


        }
        //画出识别到的二维码
        cv::aruco::drawDetectedMarkers(img,markerCorners,markerids);
        //计算算法运行时间
        clock_t finish=clock();
        double time=(finish-start)/1000;
        std::cout<<"time="<<time<<std::endl;

        cv::imshow("test",img);
        cv::waitKey(1);
        loopRate.sleep();
    }
    capture.release();
}
