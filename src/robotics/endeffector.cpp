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

    Eigen::Matrix4f ml = mTFrame2EE.inverse();

    mTwist = mathfunction::adjoint(ml) * mpFrame.lock()->get_Twist();

    // std::cout << "first term\n" << mathfunction::adjoint(ml) * mpFrame.lock()->get_Twistd() << "\n---" << std::endl;

    Eigen::Vector3f vec = ml.block<3,3>(0,0) * mTFrame2EE.block<3,1>(0,3);
    Eigen::Vector3f rot = mTwist.block<3,1>(0,0);
    // std::cout << "second term\n" << mathfunction::skew(rot) * mathfunction::skew(rot) * vec  << "\n---" << std::endl;

    mTwistd = mathfunction::adjoint(ml) * mpFrame.lock()->get_Twistd();
    mTwistd.block<3,1>(3,0) += mathfunction::skew(rot) * mathfunction::skew(rot) * vec;
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

Eigen::VectorXf EndEffector::get_Twist()
{
    update();
    return mTwist;
}

Eigen::VectorXf EndEffector::get_Twistd()
{
    update();
    return mTwistd;
}

Eigen::VectorXf EndEffector::get_GlobalTwist()
{
    Eigen::Matrix4f mT = mTGlobal;
    Eigen::VectorXf vec(6);
    vec.block<3,1>(0,0) = mTGlobal.block<3,3>(0,0) * mTwist.block<3,1>(0,0);
    vec.block<3,1>(3,0) = mTGlobal.block<3,3>(0,0) * mTwist.block<3,1>(3,0);
    return vec;
}

Eigen::VectorXf EndEffector::get_GlobalTwistd()
{
    Eigen::Matrix4f mT = mTGlobal;
    Eigen::VectorXf vec(6);
    vec.block<3,1>(0,0) = mTGlobal.block<3,3>(0,0) * mTwistd.block<3,1>(0,0);
    vec.block<3,1>(3,0) = mTGlobal.block<3,3>(0,0) * mTwistd.block<3,1>(3,0);
    return vec;
}

