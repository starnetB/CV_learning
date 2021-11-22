#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

//本程序演示了Eigen几何模块的使用方法

int main(int argc,char **argv){
    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Matrix3cd rotation_matrix=Matrix3d::Identity();
    //旋转向量使用AngleAxis,它底层不直接是Matrix,但运算可以当作矩阵（因为重载了运算符）
    AngleAxisd rotation_vector(M_PI/4,Vector3d(0,0,1));
    cout.precision(3);
    cout<<"rotation matrix=\n"<< rotation_vector.matrix()<<endl;//用matrix()转换成矩阵也可以直接赋值

    //将旋转量变成旋转矩阵,直接赋值就可以了
    rotation_matrix=rotation_vector.toRotationMatrix();

    //用AngleAxis也可以直接进行坐标变换
    Vector3d v(1,0,0);
    Vector3d v_rotated=rotation_vector*v;
    cout<<"(1,0,0) after rotation (by angle axis) = "<<v_rotated.transpose()<<endl;
    //或者可以直接用旋转矩阵
    v_rotated=rotation_matrix*v;
    cout<<"(1,0,0) after rotation (by matrix) = "<<v_rotated.transpose()<<endl;

    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即roll pitch yaw顺序
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    //欧式变换矩阵使用Eigen::Isometry
    Isometry3d T=Isometry3d::Identity();   //虽然称为3D，实质上是4*4的矩阵
    T.rotate(rotation_vector);  // 按rotation_vector进行旋转
    T.pretranslate(Vector3d(1,3,4));  //把平移向量设置成为(1,3,4)
    cout<<"Transform matrix=\n"<<T.matrix()<<endl;

    //用变换矩阵进行坐标变换
    Vector3d v_transformed=T*v;   //相当于R*v+t
    cout<<"v transformed = "<<v_transformed.transpose()<<endl;    
}