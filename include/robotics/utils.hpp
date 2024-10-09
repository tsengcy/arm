#ifndef __UTILS_HPP_
#define __UTILS_HPP_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace mathfunction
{
    std::vector<float> EigenVectorToStdVector(Eigen::VectorXf vec);

    Eigen::VectorXf PoseToPosOri(Eigen::Matrix4f Pose);
    
    Eigen::Vector3f PoseToPos(Eigen::Matrix4f Pose);
    
    Eigen::Matrix4f PosOriToPose(Eigen::VectorXf PosOri);
    
    Eigen::Matrix3f so3ToSO3(Eigen::Vector3f so);
    
    Eigen::Vector3f SO3Toso3(Eigen::Matrix3f SO);
    
    Eigen::Matrix4f se3ToSE3(Eigen::VectorXf se);
    
    Eigen::VectorXf SE3Tose3(Eigen::Matrix4f SE);
    
    void se3ToScrew(Eigen::VectorXf se, Eigen::VectorXf& Screw, float& theta);
    
    void SE3ToScrew(Eigen::Matrix4f SE, Eigen::VectorXf& Screw, float& theta);
    
    Eigen::Matrix3f skew(Eigen::Vector3f vec);

    Eigen::Matrix4f skew(Eigen::VectorXf vec);
    
    Eigen::Vector3f nskew(Eigen::Matrix3f mat);
    
    Eigen::MatrixXf adjoint(Eigen::VectorXf PosOri);
    
    Eigen::MatrixXf adjoint(Eigen::Matrix4f Pose);
    
    Eigen::MatrixXf LieBracket(Eigen::VectorXf v1);
};

#endif // __UTILS_HPP_