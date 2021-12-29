#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
string image_file="./distorted.png";

int main(int argc,char **argv){
    //本程序实现去畸变的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解
    //畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image=cv::imread(image_file,0); //图像是会灰度图，CV_8UC1
    int rows=image.rows,cols=image.cols;
    cv::Mat image_undistort =cv::Mat(rows,cols,CV_8UC1); //去畸变以后的图

    //计算去畸变后的图像内容
    for(int v=0;v<rows;v++){
        for(int u=0;u<cols;u++){
            //按照公式,计算点(u,v)对应到畸变图像中的坐标(u_distorted,v_distored)
            double x=(u-cx)/fx,y=(v-cy)/fy; //将点转换到相机坐标系中
            double r=sqrt(x*x+y*y);
            double x_distorted=x*(1+k1*r*r+k2*r*r*r*r)+2*p1*x*y+p2*(r*r+2*x*x);
            double y_distorted=y*(1+k1*r*r+k2*r*r*r*r)+p1*(r*r+2*y*y)+2*p2*x*y;
            double u_distorted=fx*x_distorted+cx;
            double v_distorted=fy*y_distorted+cy;

            //赋值 (最邻近插值)
            if((u_distorted>=0 &&v_distorted>=0) && u_distorted < cols && v_distorted < rows){  //确保畸变后俄像素位置在合理的范围内
                //将像素点畸变后位置与未畸变后的位置对应起来，然后将畸变后的位置变回到原来的位置处
                image_undistort.at<uchar>(v,u) =image.at<uchar>((int)v_distorted,(int)u_distorted);
            }else{
                image_undistort.at<uchar>(v,u)=0;
            }
        }
    }
    //画出去畸变后的图像
    cv::imshow("distorted",image);
    cv::imshow("undistorted",image_undistort);
    cv::waitKey();
    return 0;
}