#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/filesystem.hpp> // includes all needed Boost.Filesystem declarations
#include <boost/algorithm/string/predicate.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tinyxml2.h>
#include <map>

//Eigen部分
#include <Eigen/Core>

//稠密矩阵的代数运算(逆，特征值)
#include <Eigen/Dense>

//Eigen几何模块 
#include <Eigen/Geometry>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>


#include <opencv/cxeigen.hpp>
#include <opencv/cv.hpp>
#include "utils/Rotation3DUtils.h"