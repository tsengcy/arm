#ifndef __DYNFRAME_HPP_
#define __DYNFRAME_HPP_

#include <robotics/frame.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <queue>

class dynFrame : public Frame
{
public:
    dynFrame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype,
             float _upperLimit, float _lowerLimit, std::shared_ptr<Frame> _parent,
             float _mass, Eigen::Matrix3f _inertia, Eigen::Matrix4f _TCOM, bool _useGravity);

    void InverseDynamicBackward();

    Eigen::VectorXf get_Force(){return mforce;}

    float get_Torque(){return mTorque;}

    void set_Backward();

    bool check_Backward();

    void addExternalForce(Eigen::VectorXf _force, Eigen::Vector3f _forcePos){mqExternalForce.push({_force, _forcePos});}

private:
    float mMass;

    Eigen::Matrix3f mInertia;

    Eigen::MatrixXf mSpatialInertia;

    Eigen::Matrix4f mTCOM; // transformation matrix from axis to COM

    int visited{0};

    Eigen::VectorXf mforce{Eigen::VectorXf::Zero(6)};

    std::queue<std::pair<Eigen::VectorXf, Eigen::Vector3f>> mqExternalForce;

    float mTorque{0};

    int mnbackward;
};

#endif //__DYNFRAME_HPP_