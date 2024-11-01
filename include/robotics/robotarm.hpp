#ifndef __ROBOTARM_HPP_
#define __ROBOTARM_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include "robotics/frame.hpp"
#include "robotics/endeffector.hpp"

extern int robotarmId;

class RobotArm
{
public:
    RobotArm();

    RobotArm(std::vector<float> _va, std::vector<float> _valpha, std::vector<float> _vd, std::vector<float> _vtheta, 
            std::vector<float> _vupperLimit, std::vector<float> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype,
            std::vector<FRAMETYPE> _vFrametype, std::vector<Eigen::Matrix4f> _vTFrame2EE, std::vector<int> _vEEparent);

    void set_q(Eigen::VectorXf& _q);

    void set_q(std::vector<float> _q);

    void set_qd(Eigen::VectorXf _qd);

    void set_qd(std::vector<float> _qd);

    void set_qdd(Eigen::VectorXf _qdd);

    void set_qdd(std::vector<float> _qdd);


    Eigen::VectorXf get_Activeq();

    Eigen::VectorXf get_Passiveq();

    Eigen::VectorXf get_q();

    Eigen::VectorXf get_Activeqd();

    Eigen::VectorXf get_Passiveqd();

    Eigen::VectorXf get_qd();

    Eigen::VectorXf get_Activeqdd();

    Eigen::VectorXf get_Passiveqdd();

    Eigen::VectorXf get_qdd();

    Eigen::MatrixXf get_EEPose();

    Eigen::VectorXf get_EEPosOri();

    Eigen::VectorXf get_EEPos();

    Eigen::VectorXf get_Twist();

    Eigen::VectorXf get_Twistd();

    Eigen::VectorXf get_GlobalTwist();

    Eigen::VectorXf get_GlobalTwistd();


    // Eigen::VectorXf IK_Pose(Eigen::MatrixXf _eePose);

    Eigen::VectorXf IK_Pos(Eigen::VectorXf _eePos);

    Eigen::VectorXf IK_PosOri(Eigen::VectorXf _eePosOri);

    void setGravity(bool _useGravity);

    void property();
protected:
    int insert_Frame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype, float _upperLimit, float _lowerLimit, int _parentId);

    void insert_endeffector(Eigen::Matrix4f _TFrame2EE, int _EEparent);

    Eigen::MatrixXf get_JacobainPos();

    Eigen::MatrixXf get_JacobainPosOri();

protected:
    std::shared_ptr<Frame> mpRoot;

    std::vector<std::weak_ptr<Frame>> mvpLeaf;

    std::vector<std::weak_ptr<Frame>> mvpActiveFrame;

    std::vector<std::weak_ptr<Frame>> mvpPassiveFrame;

    std::vector<std::weak_ptr<Frame>> mvpFrame;

    std::vector<std::shared_ptr<EndEffector>> mvpEE;

    int mnDoF{0};

    int mnPassiveDoF{0};

    int mnid;

    int mnEE{0};
};

#endif // __ROBOTARM_HPP_