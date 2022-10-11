#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc,char **argv)
{
    double ar=1.0,br=2.0,cr=1.0;    //真实参数值
    double ae=2.0,be=-1.0,ce=5.0;  //估计参数值
    int N=100;                      //需要100个数据点
    double w_sigma=1.0;             //标准差 sigma的值
    double inv_sigma=1.0/w_sigma;   //标准差的倒数
    cv::RNG rng;                    //这个是OpenCV随机数产生器

    vector<double> x_data,y_data;     //数据
    for(int i=0;i<N;i++){
        double x=i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x+br*x+cr)+rng.gaussian(w_sigma*w_sigma));
    }  //这边的话真值产生完毕
}