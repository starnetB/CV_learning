#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>

using namespace std;

typedef vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;


//Camera intrinsics 
double fx=718.856,fy=718.856,cx=607.1928,cy=185.2157; 

//baseline
double baseline=0.573;