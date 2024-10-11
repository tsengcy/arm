#ifndef __DYNROBOTARM_HPP_
#define __DYNROBOTARM_HPP_

#include <robotics/robotarm.hpp>
#include <robotics/dynframe.hpp>
#include <robotics/utils.hpp>
#include <vector>
#include <memory>
#include <queue>

class dynRobotArm : public RobotArm
{
public:
    dynRobotArm(std::vector<float> _va, std::vector<float> _valpha, std::vector<float> _vd, std::vector<float> _vtheta, 
                std::vector<float> _vupperLimit, std::vector<float> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype,
                std::vector<FRAMETYPE> _vFrametype, std::vector<float> _vmass, std::vector<Eigen::Matrix3f> _vinertira, std::vector<Eigen::Matrix4f> _vTCOM,
                std::vector<Eigen::Matrix4f> _vTFrame2EE, std::vector<int> _vEEparent, bool _useGravity);

    void Forward_Dynamic();

    void Inverse_Dynamic(Eigen::VectorXf _q, Eigen::VectorXf _qd, Eigen::VectorXf _qdd,
                         std::vector<Eigen::VectorXf> _externalForce = std::vector<Eigen::VectorXf>(), std::vector<Eigen::Vector3f> _externalForcePose = std::vector<Eigen::Vector3f>(), std::vector<int> _externalForceId = std::vector<int>());

    Eigen::VectorXf get_Torque();
private:
    void insert_DynFrame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype, float _upperLimit, float _lowerLimit, int _parentId,
                         float _mass, Eigen::Matrix3f _inertira, Eigen::Matrix4f _TCOM, bool _useGravity);
private:

};

#endif