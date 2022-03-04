#include "DBoW3/DBoW3.h"   //运用K-mean算法训练词语
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>


using namespace cv;
using namespace std;


/***************************************************
 * 本节演示了如何根据data/目录下的十张图训练字典
 * ************************************************/

int main(int argc,char **argv){
    //read the image
    cout<<"read images..."<<endl;
    vector<Mat> images;
    for (int i=0;i<10;i++)
    {
        string path = "./data/"+to_string(i+1)+".png";
        images.push_back( imread(path) );
    }
    // detect ORB features
    cout<<"detecting ORB features ... "<<endl;
    Ptr< Feature2D > detector = ORB::create();
    vector<Mat> descriptors;

    for ( Mat& image:images )
    {
        vector<KeyPoint> keypoints; 
        Mat descriptor;
        //从图像image中提取关键点和描述子
        //最后将描述子保存起来
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
    }
    // create vocabulary 
    cout<<"creating vocabulary ... "<<endl;
    DBoW3::Vocabulary vocab;
    //使用K-mean方法，来创建字典，含有k个类，中间过程使用了k叉树
    //k=10,d=5默认
    vocab.create( descriptors );
    cout<<"vocabulary info: "<<vocab<<endl;  
    vocab.save( "vocabulary.yml.gz" );
    cout<<"done"<<endl;
    
    return 0;
}