#include <robotics/dynframe.hpp>

template<typename T>
dynFrame<T>::dynFrame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype,
                      T _upperLimit, T _lowerLimit, std::shared_ptr<Frame<T>> _parent,
                      T _mass, Eigen::Matrix<T, 3, 3> _inertia, Eigen::Matrix<T, 4, 4> _TCOM)
                      : mMass(_mass), mInertia(_inertia), mTCOM(_TCOM), 
                      Frame<T>(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, _parent)
{
    mSpatialInertia = Eigen::Matrix<T, -1, -1>::Identity(6, 6) * mMass;
    mSpatialInertia.block(0, 0, 3, 3) = mInertia;

    Eigen::Matrix<T, 4, 4> mT = mTCOM.inverse();

    Eigen::Matrix<T, -1, -1> ad = mathfunction::adjoint(mT);
    mSpatialInertia = ad.transpose() * mSpatialInertia * ad;
#ifdef DEBUG
    std::cout << "spitial Inertia\n" << mSpatialInertia << std::endl; 
#endif
    
}

template<typename T>
void dynFrame<T>::InverseDynamicBackward()
{
    Eigen::Matrix<T, -1, 1> screw;
    T degree;
    mathfunction::SE3ToScrew(Frame<T>::mglobal, screw, degree);
    mforce = Eigen::Matrix<T, -1, 1>::Zero(6);
    for(int i=0; i<Frame<T>::mvpChildren.size(); i++)
    {
        std::shared_ptr<dynFrame<T>> nf= std::dynamic_pointer_cast<dynFrame<T>>(Frame<T>::mvpChildren[i]);
        Eigen::Matrix<T, 4, 4> Tlocal = Frame<T>::mlocal.inverse();
        mforce = mforce + mathfunction::adjoint(Tlocal) * nf->get_Force();
    }

    mforce = mforce + mSpatialInertia * Frame<T>::mTwistsd - mathfunction::LieBracket(Frame<T>::mTwists).transpose() * (mSpatialInertia * Frame<T>::mTwists);

    mTorque = mforce.dot(screw);
}



