#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc,char **argv)
{
    //读取argv[1]指定的图像
    cv::Mat image;
    image=cv::imread(argv[1]);  //cv::imread 函数读取指定路径下的图像

    //判断图像文件是否正确读取
    if(image.data==nullptr){  //数据不存在，或者是文件不存在
        cerr<<"文件"<<argv[1]<<"不存在"<<endl;
        return 0;
    }

    //文件顺序读取，首先输出一些基本信息
    cout<<"图像宽为"<<image.cols<<",高为"<<image.rows
        <<",通道数为"<<image.channels()<<endl;

    cv::imshow("image",image);   //用cv::imshow 显示图像
    cv::waitKey(0);              //暂停程序
}

