#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

using namespace std;
using namespace cv;

/** @brief Class for matching keypoint descriptors
query descriptor index, train descriptor index, train image index, and distance between
descriptors.
*/
// class CV_EXPORTS_W_SIMPLE DMatch
// {
// public:
//     CV_WRAP DMatch();//默认构造函数

//     CV_WRAP DMatch(int _queryIdx, int _trainIdx, float _distance);//构造函数
//     CV_WRAP DMatch(int _queryIdx, int _trainIdx, int _imgIdx, float _distance);
 
//     CV_PROP_RW int queryIdx; //query描述子下标,即match函数中位于前面的描述子
//     CV_PROP_RW int trainIdx; // train描述子下标，match函数中位于后面的描述子
 
//    // 匹配图像下标，对于一副图像img1的描述子,在其他图像中找到与其最相似的，imgIdx就是找到的图像下标
//     CV_PROP_RW int imgIdx;  
 
//     //两个描述子的距离，一般是欧式距离
//     CV_PROP_RW float distance;
//     // less is better
//     bool operator<(const DMatch &m) const;
// };
/****************************************************
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 * **************************************************/


void find_feature_matches(const Mat &img_1,const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches);


void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R,Mat &t);

//像素坐标转相机归一化坐标

Point2d pixel2cam(const Point2d &p, const Mat &K);

int main(int argc,char **argv){
    if (argc != 3) {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }
    //读取图像
    Mat img_1=imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2=imread(argv[2],CV_LOAD_IMAGE_COLOR);
    assert(img_1.data &&img_2.data && "Can not load images!");

    //提取关键点  
    vector<KeyPoint> keypoints_1,keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    //--估计两张图像间运动  
    Mat R,t;
    pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);

    //--验证 E=t^ R *scale
    Mat t_x=(Mat_<double>(3,3)<<0,-t.at<double>(2,0),t.at<double>(1,0),
                                t.at<double>(2,0),0,-t.at<double>(0,0)
                                -t.at<double>(1,0),t.at<double>(0,0),0);
    cout << "t^R=" << endl << t_x * R << endl;

    //--验证对极约束   
    Mat K=(Mat_<double>(3,3)<<520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (DMatch m:matches){
        Point2d pt1=pixel2cam(keypoints_1[m.queryIdx].pt,K);
        Mat y1=(Mat_<double>(3,1)<<pt1.x,pt1.y,1);
        Point2d pt2=pixel2cam(keypoints_2[m.trainIdx].pt,K);
        Mat y2=(Mat_<double>(3,1)<<pt2.x,pt2.y,1);
        Mat d=y2.t()*t_x*R*y1;
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}

void find_feature_matches(const Mat &img_1,const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches){
    //-- 初始化 
    Mat descriptors_1,descriptors_2;
    // used in Opencv3
    Ptr<FeatureDetector> detector =ORB::create();
    Ptr<DescriptorExtractor> descriptor=ORB::create();

    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create("BruteForce-Hamming:");
    //--第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1,keypoints_1,descriptors_1);
    descriptor->compute(img_2,keypoints_2,descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1,descriptors_2,match);

     //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;
    for(int i=0;i<descriptors_1.rows;i++){
        double dist=match[i].distance;
        if(dist<min_dist) min_dist=dist;
        if(dist>max_dist) max_dist=dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for(int i=0;i<descriptors_1.rows;i++){
        if(match[i].distance<=max(2*min_dist,30.0)){
            matches.push_back(match[i]);
        }
    }
}

Point2d pixel2cam(const Point2d &p,const Mat &K){
    return Point2d
        (
            //u-c_x/f_x=X/Z
            //v-c_y/f_y=Y/Z
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        );
}

void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R,Mat &t){
    // 相机内参,TUM Freiburg2
    Mat K=(Mat_<double>(3,3)<<520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //--把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //--计算本质矩阵  
    Point2d principal_point(325.1,249.7);  //相机光心, TUM dataset标定值
    double focal_length=521;    //相机焦距，TUM dataset标定值  
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 计算单应矩阵
    //-- 但是本例中场景不是平面，单应矩阵意义不大
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "homography_matrix is " << endl << homography_matrix << endl;


    //-- 从本质矩阵中恢复旋转和平移信息
    // 此函数仅在Opencv3中提供
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}