#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <cstdlib>
#include <pcl/io/io.h>

using namespace std;
using namespace cv;

float qnan_=std::numeric_limits<float>::quiet_NaN();
const char *cameraInCailFile = "./assets/3DCameraInCailResult.xml";

Eigen::Matrix<float,1920,1> colmap;
Eigen::Matrix<float,1080,1> rowmap;

//const short w=512,h=424;
const short w = 1920, h = 1080;

void prepareMake3D(const double cx,const double cy,
                   const double fx,const double fy){
    float *pm1=colmap.data();
    float *pm2=rowmap.data();

    for (int i = 0; i < w; i++) {
        *pm1++ = (i - cx + 0.5) / fx;
    }
    for (int i = 0; i < h; i++) {
        *pm2++ = (i - cy + 0.5) / fy;
    }
}
/**
 * 根据内参，合并RGB彩色图和深度图到点云
 * @param cloud
 * @param depthMat
 * @param rgbMat
 */

void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,Mat &depthMat,Mat &rgbMat){
    const float *itD0 = (float *)depthMat.ptr();
    const char *itRGB0 = (char *)rgbMat.ptr();

    if (cloud->size() != w * h)
        cloud->resize(w * h);

    pcl::PointXYZRGB *itP = &cloud->points[0];
    bool is_dense = true;

    for (size_t y = 0; y < h; ++y) {
        const unsigned int offset = y * w;
        const float *itD = itD0 + offset;      //固定到某一行
        const char *itRGB = itRGB0 + offset * 4;   //固定到某一行
        const float dy=rowmap(y);

        for (size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4) {
            const float depth_value=*itD/1000.0f;

            if(!isnan(depth_value)&&abs(depth_value)>=0.0001){
                const float rx=colmap(x)*depth_value;
                const float ry=dy*depth_value;

                itP->z=depth_value;
                itP->x=rx;
                itP->y=ry;

                itP->b=itRGB[0];
                itP->g=itRGB[1];
                itP->r=itRGB[2];
            }else{
                itP->z=qnan_;
                itP->x=qnan_;
                itP->y=qnan_;

                itP->b=qnan_;
                itP->g=qnan_;
                itP->r=qnan_;
                is_dense=false;
            }
        }   
    }
    cloud->is_dense=is_dense;
}   

int main(){
    Mat cameraMatrix=cv::Mat_<double>(3,3);
    FileStorage paramFs(cameraInCailFile,FileStorage::READ);
    paramFs["cameraMatrix"]>>cameraMatrix;

    //内参数据
    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double cx=cameraMatrix.at<double>(0,2);
    double cy=cameraMatrix.at<double>(1,2);

    //提前准备计算所需参数
    prepareMake3D(cx,cy,fx,fy);

    cv::Mat rgbMat;   //从相机得到RGB彩色图
    cv::Mat depthMat;  //从相机得到depth深度图
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    getCloud(cloud,depthMat,rgbMat);
}

