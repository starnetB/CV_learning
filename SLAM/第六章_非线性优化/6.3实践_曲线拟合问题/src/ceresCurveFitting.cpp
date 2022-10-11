#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

//代价函数的计算模型
struct CURVE_FITTING_COST{
    CURVE_FITTING_COST(double x,double y):_x(x),_y(y){}
    
    //残差的计算  重载操作符（）
    template<typename T>
    bool operator()(
        const T *const abc,  //这是一个指向模板参数的常量指针,我们需要的模板参数T，有三维
        T *residual) const{
        //y=exp(ax^2+bx+c)
        residual[0]=T(_y)-ceres::exp(abc[0]*T(_x)*T(_x)+abc[1]*T(_x)+abc[2]);
        return true;
        }
    const double _x,_y;
};

int main(int argc,char **argv){
    double ar=1.0,br=2.0,cr=1.0;      //真实参数值
    double ae=2.0,be=-1.0,ce=5.0;     //估计参数值
    int N=100;                        //数据点数量
    double w_sigma=1.0;               //噪声Sigma值
    double inv_sigma=1.0/w_sigma;     
    cv::RNG rng;                      //OpenCV随机数产生器

    vector<double> x_data,y_data;  //数据
    for(int i=0;i<N;i++){
        double x=i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x+br*x+cr)+rng.gaussian(w_sigma*w_sigma));
    }

    double abc[3]={ae,be,ce};

    //构建最小二乘问题
    ceres::Problem problem;

    for(int i=0;i<N;i++){
        problem.AddResidualBlock(    //向问题中添加误差项
            //使用自动求到，模板参数：误差类型、输出维度、输入维度、
            //维数要与前面的struct中的一致，
            //这里输入是abc是一个三维度的数组，输出的残差res，是一个1个维度的数组，所以是1，3
            //N=100，每一个数据都会分配一个，残差块，最终残差值在平方传给 roll核函数之后，相加
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
                new CURVE_FITTING_COST(x_data[i],y_data[i])
            ),
            nullptr,              //核函数，这里不使用，为空
            abc                   //带估计参数，是三维的
        );
    }

    //配置求解器
    ceres::Solver::Options option;  //这里有很多配置项可以填写
    option.linear_solver_type=ceres::DENSE_NORMAL_CHOLESKY; //增量方法如何求解
    option.minimizer_progress_to_stdout=true;   //输出到cout 

    ceres::Solver::Summary summary;    //优化信息内容
    chrono::steady_clock::time_point t1=chrono::steady_clock::now();
    ceres::Solve(option,&problem,&summary);//开始初始化
    chrono::steady_clock::time_point t2=chrono::steady_clock::now();
chrono::duration<double> time_used=chrono::duration_cast<chrono::duration<double>>(t2-t1);    chrono::duration<double> time_used=chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    //输出结果
    cout<<summary.BriefReport()<<endl;
    cout<<" estimated a,b,c=";
    for (auto a:abc) cout<<a<<" ";
    cout<<endl;

    return 0;
}