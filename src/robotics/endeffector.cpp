#include <robotics/endeffector.hpp>

int EEId = 0;

EndEffector::EndEffector(std::shared_ptr<Frame> frame, Eigen::Matrix4f trans)
    : mpFrame(frame), mTFrame2EE(trans)
{
    mnid = EEId++;
}

void EndEffector::update()
{
    mTGlobal = mpFrame.lock()->get_GlobalPose() * mTFrame2EE;
}

Eigen::Matrix4f EndEffector::get_GlobalPose()
{
    update();
    return mTGlobal;
}

Eigen::VectorXf EndEffector::get_GlobalPosOri()
{
    update();
    return mathfunction::PoseToPosOri(mTGlobal);
}

Eigen::Vector3f EndEffector::get_GlobalPos()
{
    update();
    return mathfunction::PoseToPos(mTGlobal);
}



