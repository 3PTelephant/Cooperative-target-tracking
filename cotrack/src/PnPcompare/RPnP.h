#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>

#include <vector>
using namespace cv;
using namespace std;

void LookMat(Mat& mat, char* str);
void Lookvalue(double& val,char* str);
void calcampose(Mat& XXc,Mat&XXw,Mat& foundR, Mat& foundT);
bool RPnP(std::vector<Point3d>& WorldPts, std::vector<Point2d>& Imgpts, Mat& rvec, Mat& tvec, Point2d& PrincipalPoint, double focallength);
