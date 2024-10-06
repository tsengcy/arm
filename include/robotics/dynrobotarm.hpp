#ifndef __DYNROBOTARM_HPP_
#define __DYNROBOTARM_HPP_

#include <robotics/robotarm.hpp>
#include <robotics/dynframe.hpp>
#include <robotics/utils.hpp>
#include <vector>
#include <memory>
#include <queue>

template<typename T>
class dynRobotArm : public RobotArm<T>
{
public:
    dynRobotArm(std::vector<T> _va, std::vector<T> _valpha, std::vector<T> _vd, std::vector<T> _vtheta, 
                std::vector<T> _vupperLimit, std::vector<T> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype,
                std::vector<FRAMETYPE> _vFrametype, std::vector<T> _vmass, std::vector<Eigen::Matrix<T, 3, 3>> _vinertira, std::vector<Eigen::Matrix<T, 4, 4>> _vTCOM,
                std::vector<Eigen::Matrix<T, 4, 4>> _vTFrame2EE, std::vector<int> _vEEparent);

    void Forward_Dynamic();

    void Inverse_Dynamic(Eigen::Matrix<T, -1, 1> _q, Eigen::Matrix<T, -1, 1> _qd, Eigen::Matrix<T, -1, 1> _qdd);

private:
    void insert_DynFrame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype, T _upperLimit, T _lowerLimit, int _parentId,
                         T _mass, Eigen::Matrix<T, 3, 3> _inertira, Eigen::Matrix<T, 4, 4> _TCOM);

private:

};

template class dynRobotArm<float>;
template class dynRobotArm<double>;

typedef dynRobotArm<float> dynRobotArmf;
typedef dynRobotArm<double> dynRobotArmd;


#endif