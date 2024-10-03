#include <robotics/endeffector.hpp>

int EEId = 0;

template<typename T>
EndEffector<T>::EndEffector(std::shared_ptr<Frame<T>> frame, Eigen::Matrix<T, 4, 4> trans)
    : mpFrame(frame), mTFrame2EE(trans)
{
    mnid = EEId++;
}

template<typename T>
void EndEffector<T>::update()
{
    mTGlobal = mpFrame.lock()->get_GlobalPose() * mTFrame2EE;
}

template<typename T>
Eigen::Matrix<T, 4, 4> EndEffector<T>::get_GlobalPose()
{
    update();
    return mTGlobal;
}

template<typename T>
Eigen::Matrix<T, -1, 1> EndEffector<T>::get_GlobalPosOri()
{
    update();
    return mathfunction::PoseToPosOri(mTGlobal);
}

template<typename T>
Eigen::Matrix<T, 3, 1> EndEffector<T>::get_GlobalPos()
{
    update();
    return mathfunction::PoseToPos(mTGlobal);
}



