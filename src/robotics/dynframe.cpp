#include <robotics/dynframe.hpp>

dynFrame::dynFrame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype,
                   float _upperLimit, float _lowerLimit, std::shared_ptr<Frame> _parent,
                   float _mass, Eigen::Matrix3f _inertia, Eigen::Matrix4f _TCOM)
                   : mMass(_mass), mInertia(_inertia), mTCOM(_TCOM), 
                   Frame(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, _parent)
{
    mSpatialInertia = Eigen::MatrixXf::Identity(6, 6) * mMass;
    mSpatialInertia.block<3,3>(0,0) = mInertia;

    Eigen::Matrix4f mT = mTCOM.inverse();

    Eigen::MatrixXf ad = mathfunction::adjoint(mT);
    mSpatialInertia = ad.transpose() * mSpatialInertia * ad;
#ifdef DEBUG
    std::cout << "spitial Inertia\n" << mSpatialInertia << std::endl; 
#endif
}

void dynFrame::InverseDynamicBackward()
{
    Eigen::VectorXf screw;
    float degree;
    mathfunction::SE3ToScrew(Frame::mglobal, screw, degree);
    mforce = Eigen::VectorXf::Zero(6);
    for(int i=0; i<Frame::mvpChildren.size(); i++)
    {
        std::shared_ptr<dynFrame> nf= std::dynamic_pointer_cast<dynFrame>(Frame::mvpChildren[i]);
        Eigen::Matrix4f Tlocal = Frame::mlocal.inverse();
        mforce = mforce + mathfunction::adjoint(Tlocal) * nf->get_Force();
    }

    mforce = mforce + mSpatialInertia * Frame::mTwistsd - mathfunction::LieBracket(Frame::mTwists).transpose() * (mSpatialInertia * Frame::mTwists);

    mTorque = mforce.dot(screw);
}



