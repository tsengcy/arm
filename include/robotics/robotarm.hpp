#ifndef __ROBOTARM_HPP_
#define __ROBOTARM_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include "robotics/frame.hpp"
#include "robotics/endeffector.hpp"

extern int robotarmId;

template<typename T>
class RobotArm
{
public:
    RobotArm();

    RobotArm(std::vector<T> _va, std::vector<T> _valpha, std::vector<T> _vd, std::vector<T> _vtheta, 
            std::vector<T> _vupperLimit, std::vector<T> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype,
            std::vector<FRAMETYPE> _vFrametype, std::vector<Eigen::Matrix<T, 4, 4>> _vTFrame2EE, std::vector<int> _vEEparent);

    void set_q(Eigen::Matrix<T, -1, 1> _q);

    void set_q(std::vector<T> _q);

    void set_qd(Eigen::Matrix<T, -1, 1> _qd);

    void set_qd(std::vector<T> _qd);

    void set_qdd(Eigen::Matrix<T, -1, 1> _qdd);

    void set_qdd(std::vector<T> _qdd);


    Eigen::Matrix<T, -1, 1> get_Activeq();

    Eigen::Matrix<T, -1, 1> get_Passiveq();

    Eigen::Matrix<T, -1, 1> get_q();

    Eigen::Matrix<T, -1, -1> get_EEPose();

    Eigen::Matrix<T, -1, 1> get_EEPosOri();

    Eigen::Matrix<T, -1, 1> get_EEPos();

    void property();
protected:
    int insert_Frame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype, T _upperLimit, T _lowerLimit, int _parentId);

    void insert_endeffector(Eigen::Matrix<T, 4, 4> _TFrame2EE, int _EEparent);

protected:
    std::shared_ptr<Frame<T>> mpRoot;

    std::vector<std::weak_ptr<Frame<T>>> mvpActiveFrame;

    std::vector<std::weak_ptr<Frame<T>>> mvpPassiveFrame;

    std::vector<std::weak_ptr<Frame<T>>> mvpFrame;

    std::vector<std::shared_ptr<EndEffector<T>>> mvpEE;

    int mnDoF{0};

    int mnPassiveDoF{0};

    int mnid;

    int mnEE{0};
};

template class RobotArm<float>;
template class RobotArm<double>;

using RobotArmf = RobotArm<float>;
using RobotArmd = RobotArm<double>;

#endif // __ROBOTARM_HPP_