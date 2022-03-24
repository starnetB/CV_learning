#include <iostream>
#include <ceres/ceres.h>
#include "common.h"
#include "SnavelyReprojectionError.h"

using namespace std;

void SolveBA(BALProplem &bal_problem);

int main(int argc,char **argv){
    if(argc!=2){
        cout<<"usage:bundle_adjustment_ceres bal_data.txt"<<endl;
        return 1;
    }

    BALProblem bal_problem(argv[1]);
    bal_problem.Normalize();  //正则化，去掉平均值，标准缩放
    bal_problem.Perturb(0.1,0.5,0.5);  //增加固定sigma的扰动
    bal_problem.WriteToPLYFile("initial.ply");
    SolveBA(bal_problem);
    bal_problem.WriteToPLYFile("final.ply");
    return 0;
}

void SolveBA(BALProplem &bal_problem){
    const int point_block_size=bal_problem.point_block_size();   //size=3
    const int camera_block_size=bal_problem.camera_block_size();  //camera_size=9
    double *points=bal_problem.mutable_points();    //指向points的初始点
    double *cameras=bal_problem.mutable_cameras();   //指向camera的初始点



    // Observations is 2 * num_observations long array observations
    // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
    // and y position of the observation.
    const double *observations=bal_problem.observations();
    ceres::Problem problem;

    for(int i=0;i<bal_problem.num_observations();++i){
        ceres::CostFunction *cost_function;  

        // Each Residual block takes a point and a camera as input
        // and outputs a 2 dimensional Residual
        cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
        
        // If enabled use Huber's loss function.
        //创建损失函数的计算方法
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

        // Each observation corresponds to a pair of a camera and a point
        // which are identified by camera_index()[i] and point_index()[i]
        // respectively.

        //创建残差块
        problem.AddResidualBlock(cost_function, loss_function, camera, point);
    }

    //show some information here ...
    // show some information here ...
    std::cout << "bal problem file loaded..." << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observations. " << std::endl;

    std::cout << "Solving ceres BA ... " << endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}