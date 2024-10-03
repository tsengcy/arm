#ifndef __ENDEFFECTOR_HPP_
#define __ENDEFFECTOR_HPP_

#include <iostream>
#include <memory>
#include "robotics/frame.hpp"
#include "robotics/utils.hpp"

extern int EEId;

template<typename T>
class EndEffector
{
public:
    EndEffector(std::shared_ptr<Frame<T>> frame, Eigen::Matrix<T, 4, 4> trans);

    Eigen::Matrix<T, 4, 4> get_GlobalPose();

    Eigen::Matrix<T, -1, 1> get_GlobalPosOri();

    Eigen::Matrix<T, 3, 1> get_GlobalPos();

    int get_Id(){return mnid;}

private:
    void update();

private:
    std::weak_ptr<Frame<T>> mpFrame;
    
    Eigen::Matrix<T, 4, 4> mTFrame2EE;

    Eigen::Matrix<T, 4, 4> mTGlobal;

    int mnid;
};

template class EndEffector<float>;
template class EndEffector<double>;

using EndEffectorf = EndEffector<float>;
using EndEffectord = EndEffector<double>;




#endif // __ENDEFFECTOR_HPP_