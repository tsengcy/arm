#ifndef __UTILS_HPP_
#define __UTILS_HPP_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace mathfunction
{
    template<typename T>
    std::vector<T> EigenVectorToStdVector(Eigen::Matrix<T, -1, 1> vec);

    template<typename T>
    Eigen::Matrix<T, -1, 1> PoseToPosOri(Eigen::Matrix<T, 4, 4> Pose);

    template<typename T>
    Eigen::Matrix<T, 3, 1> PoseToPos(Eigen::Matrix<T, 4, 4> Pose);

    template<typename T>
    Eigen::Matrix<T, 4, 4> PosOriToPose(Eigen::Matrix<T, -1, 1> PosOri);

    template<typename T>
    Eigen::Matrix<T, 3, 3> so3ToSO3(Eigen::Matrix<T, 3, 1> so);

    template<typename T>
    Eigen::Matrix<T, 3, 1> SO3Toso3(Eigen::Matrix<T, 3, 3> SO);

    template<typename T>
    Eigen::Matrix<T, 4, 4> se3ToSE3(Eigen::Matrix<T, -1, 1> se);

    template<typename T>
    Eigen::Matrix<T, -1, 1> SE3TOse3(Eigen::Matrix<T, 4, 4> SE);

    template<typename T>
    Eigen::Matrix<T, 3, 3> skew(Eigen::Matrix<T, 3, 1> vec);

    template<typename T>
    Eigen::Matrix<T, 3, 1> nskew(Eigen::Matrix<T, 3, 3> mat);

    template<typename T>
    Eigen::Matrix<T, 3, 3> transpose(Eigen::Matrix<T, 3, 3> mat);
};

template<typename T>
std::vector<T> mathfunction::EigenVectorToStdVector(Eigen::Matrix<T, -1, 1> vec)
{
    std::vector<T> ans(vec.rows(), 0);
    for(int i=0; i<vec.rows(); i++)
    {
        ans[i] = vec(i);
    }
    return ans;
}

template<typename T>
Eigen::Matrix<T, -1, 1> mathfunction::PoseToPosOri(Eigen::Matrix<T, 4, 4> Pose)
{
    Eigen::Matrix<T, -1, 1> PosOri(6);
    Eigen::Matrix<T, 3, 3> mat = Pose.block(0, 0, 3, 3);
    PosOri.block(0, 0, 3, 1) = SO3Toso3(mat);
    PosOri.block(3, 0, 3, 1) = Pose.block(0, 3, 3, 1);

    return PosOri;
}

template<typename T>
Eigen::Matrix<T, 3, 1> mathfunction::PoseToPos(Eigen::Matrix<T, 4, 4> Pose)
{
    return Pose.block(0, 3, 3, 1);
}

template<typename T>
Eigen::Matrix<T, 4, 4> mathfunction::PosOriToPose(Eigen::Matrix<T, -1, 1> PosOri)
{
    Eigen::Matrix<T, 4, 4> Pose = Eigen::Matrix<T, 4, 4>::Identity();
    Pose.block(0, 0, 3, 3) = so3ToSO3(PosOri.block(0, 0, 3, 1));
    Pose.block(0, 3, 3, 1) = PosOri.block(3, 0, 3, 1);

    return Pose;
}

template<typename T>
Eigen::Matrix<T, 3, 3> mathfunction::so3ToSO3(Eigen::Matrix<T, 3, 1> so)
{
    T angle = so.norm();
    Eigen::Matrix<T, 3, 1> axis = so / angle;

    Eigen::Matrix<T, 3, 3> ans; 
    ans << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    ans = ans + sin(angle) * skew(axis) + (1 - cos(angle)) * skew(axis) * skew(axis);

    return ans;
}

template<typename T>
Eigen::Matrix<T, 3, 1> mathfunction::SO3Toso3(Eigen::Matrix<T, 3, 3> SO)
{
    T angle = acos((SO.trace() - 1) / 2.0);
    if(angle > M_PI || angle < -M_PI) throw std::invalid_argument("error of rotation angle");
    if(angle < 0.0001)
    {
        Eigen::Matrix<T, 3, 1> axis;
        axis << 0,0,0;
        return axis;   
    }
    else
    {
        Eigen::Matrix<T, 3, 3> mat = (SO - mathfunction::transpose(SO))/(2 * sin(angle));
        Eigen::Matrix<T, 3, 1> axis = mathfunction::nskew(mat);
        return axis * angle;
    }
}

template<typename T>
Eigen::Matrix<T, 4, 4> mathfunction::se3ToSE3(Eigen::Matrix<T, -1, 1> se)
{
    Eigen::Matrix<T, 4, 4> ans = Eigen::Matrix<T, 4, 4>::Identity();
    Eigen::Matrix<T, 3, 1> axis = se.block(0, 0, 3, 1);
    ans.block(0, 0, 3, 3) = so3ToSO3(axis);

    T angle = axis.norm();
    axis = axis / angle;

    Eigen::Matrix<T, 3, 3> G = Eigen::Matrix<T, 3, 3>::Identity() * angle 
                               + (1 - cos(angle)) * skew(axis) + (angle - sin(angle)) * skew(axis) * skew(axis);
    ans.block(0, 3, 3, 1) = G * se.block(3, 0, 3, 1);

    return ans;
}

template<typename T>
Eigen::Matrix<T, -1, 1> mathfunction::SE3TOse3(Eigen::Matrix<T, 4, 4> SE)
{
    Eigen::Matrix<T, -1, 1> ans = Eigen::Matrix<T, -1, 1>::Zero(6);
    Eigen::Matrix<T, 3, 3> mat = SE.block(0,0,3,3);
    ans.block(0, 0, 3, 1) = SO3Toso3(mat);

    T angle = ans.block(0, 0, 3, 1).norm();
    Eigen::Matrix<T, 3, 1> axis = ans.block(0, 0, 3, 1) / angle;

    Eigen::Matrix<T, 3, 3> G = Eigen::Matrix<T, 3, 3>::Identity() * angle 
                               + (1 - cos(angle)) * skew(axis) + (angle - sin(angle)) * skew(axis) * skew(axis);

    ans.block(3, 0, 3, 1) = G.inverse() * SE.block(0, 3, 3, 1);
    return ans;
}

template<typename T>
Eigen::Matrix<T, 3, 3> mathfunction::skew(Eigen::Matrix<T, 3, 1> vec)
{
    Eigen::Matrix<T, 3, 3> ans;
    ans <<       0, -vec(2),  vec(1),
            vec(2),       0, -vec(0),
           -vec(1),  vec(0),       0;
    return ans;
}

template<typename T>
Eigen::Matrix<T, 3, 1> mathfunction::nskew(Eigen::Matrix<T, 3, 3> mat)
{
    Eigen::Matrix<T, 3, 1> vec;
    vec << mat(2, 1), mat(0, 2), mat(1, 0);

    return vec; 
}

template<typename T>
Eigen::Matrix<T, 3, 3> mathfunction::transpose(Eigen::Matrix<T, 3, 3> mat)
{
    Eigen::Matrix<T, 3, 3> ans;
    ans << mat(0, 0), mat(1, 0), mat(2, 0),
           mat(0, 1), mat(1, 1), mat(2, 1),
           mat(0, 2), mat(1, 2), mat(2, 2);
    return ans;
}

#endif // __UTILS_HPP_