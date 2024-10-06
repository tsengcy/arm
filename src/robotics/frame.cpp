#include <robotics/frame.hpp>

int FrameId = 0;
int ActiveFrameId = 0;
int PassiveFrameId = 0;

template<typename T>
Frame<T>::Frame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype, 
                T _upperLimit, T _lowerLimit, std::shared_ptr<Frame<T>> _parent) 
                : ma(_a), malpha(_alpha), md(_d), mtheta(_theta), mDHtype(_DHtype),
                  mframetype(_frametype), mupperLimit(_upperLimit), mlowerLimit(_lowerLimit)
{
    mnid = FrameId++;

    if(_frametype == FRAMETYPE::ACTIVE)
    {
        mnJointId = ActiveFrameId++;
    }
    else if(_frametype == FRAMETYPE::PASSIVE)
    {
        mnJointId = PassiveFrameId++;
    }
    else
    {
        mnJointId = mnid - ActiveFrameId - PassiveFrameId;
    }

    if(_parent != nullptr)
    {
#ifdef DEBUG
        std::cout << mnid << ": Debug" << std::endl;
#endif
        mpParent = _parent;
        // _parent->set_Child(this->shared_from_this());
    }
    else
    {
        mbase = Eigen::Matrix<T, 4, 4>::Identity();
    }
    mq = 0;
}

template<typename T>
Frame<T>::~Frame()
{
    std::cout << "reset id: " << mnid << std::endl;
    for(auto &ptr : mvpChildren)
    {
        ptr.reset();
    }
}

template<typename T>
void Frame<T>::set_q(T _angle)
{
    if(mframetype == FRAMETYPE::ACTIVE)
        mq = _angle;
    else
    {
        std::stringstream ss;
        ss << "ERROR: frame " << mnid << " call a wrong set angle method\n";
        throw std::invalid_argument(ss.str());
    }
}

template<typename T>
void Frame<T>::set_qd(T _qd)
{
    if(mframetype == FRAMETYPE::ACTIVE)
        mqd = _qd;
    else
    {
        std::stringstream ss;
        ss << "ERROR: frame " << mnid << " call a wrong set angle method\n";
        throw std::invalid_argument(ss.str());
    }
}

template<typename T>
void Frame<T>::set_qdd(T _qdd)
{
    if(mframetype == FRAMETYPE::ACTIVE)
        mqdd = _qdd;
    else
    {
        std::stringstream ss;
        ss << "ERROR: frame " << mnid << " call a wrong set angle method\n";
        throw std::invalid_argument(ss.str());
    }
}

template<typename T>
void Frame<T>::set_q()
{
    if(mframetype == FRAMETYPE::PASSIVE)
    {
        if(mpRef.lock() == nullptr)
        {
            std::stringstream ss;
            ss << "ERROR: frame " << mnid << " do not set reference frame\n";
            throw std::invalid_argument(ss.str());
        }
        else
        {
            mq = mPassiveRate * mpRef.lock()->get_q();
        }
    }
    else
    {
        std::stringstream ss;
        ss << "ERROR: frame " << mnid << " call a wrong set angle method\n";
        throw std::invalid_argument(ss.str());
    }
}

template<typename T>
void Frame<T>::set_qd()
{
    if(mframetype == FRAMETYPE::PASSIVE)
    {
        if(mpRef.lock() == nullptr)
        {
            std::stringstream ss;
            ss << "ERROR: frame " << mnid << " do not set reference frame\n";
            throw std::invalid_argument(ss.str());
        }
        else
        {
            mqd = mPassiveRate * mpRef.lock()->get_qd();
        }
    }
    else
    {
        std::stringstream ss;
        ss << "ERROR: frame " << mnid << " call a wrong set angle method\n";
        throw std::invalid_argument(ss.str());
    }
}

template<typename T>
void Frame<T>::set_qdd()
{
    if(mframetype == FRAMETYPE::PASSIVE)
    {
        if(mpRef.lock() == nullptr)
        {
            std::stringstream ss;
            ss << "ERROR: frame " << mnid << " do not set reference frame\n";
            throw std::invalid_argument(ss.str());
        }
        else
        {
            mqdd = mPassiveRate * mpRef.lock()->get_qdd();
        }
    }
    else
    {
        std::stringstream ss;
        ss << "ERROR: frame " << mnid << " call a wrong set angle method\n";
        throw std::invalid_argument(ss.str());
    }
}

template<typename T>
void Frame<T>::update()
{
    if(!isRoot())
    {
        mbase = mpParent.lock()->get_GlobalPose();
    }

    if(mDHtype == DH::MODIFIED)
    {
        mlocal <<             cos(mtheta+mq),            -sin(mtheta+mq),            0,              ma,
                  sin(mtheta+mq)*cos(malpha), cos(mtheta+mq)*cos(malpha), -sin(malpha), -md*sin(malpha),
                  sin(mtheta+mq)*sin(malpha), cos(mtheta+mq)*sin(malpha),  cos(malpha),  md*cos(malpha),
                                           0,                          0,            0,               1;
    }
    else
    {
        mlocal << cos(mtheta+mq), -sin(mtheta+mq)*cos(malpha),  sin(mtheta+mq)*sin(malpha),  ma*cos(mtheta+mq),
                  sin(mtheta+mq),  cos(mtheta+mq)*cos(malpha), -cos(mtheta+mq)*sin(malpha), -ma*sin(mtheta+mq),
                               0,                 sin(malpha),                 cos(malpha),     md*cos(malpha),
                               0,                           0,                           0,                  1;
    }
    
    mglobal = mbase * mlocal;
    for(auto &child : mvpChildren)
    {
        child->update();
    }
}

template<typename T>
void Frame<T>::update2()
{
    if(!isRoot())
    {
        mbase = mpParent.lock()->get_GlobalPose();
    }

    if(mDHtype == DH::MODIFIED)
    {
        mlocal <<             cos(mtheta+mq),            -sin(mtheta+mq),            0,              ma,
                  sin(mtheta+mq)*cos(malpha), cos(mtheta+mq)*cos(malpha), -sin(malpha), -md*sin(malpha),
                  sin(mtheta+mq)*sin(malpha), cos(mtheta+mq)*sin(malpha),  cos(malpha),  md*cos(malpha),
                                           0,                          0,            0,               1;
    }
    else
    {
        mlocal << cos(mtheta+mq), -sin(mtheta+mq)*cos(malpha),  sin(mtheta+mq)*sin(malpha),  ma*cos(mtheta+mq),
                  sin(mtheta+mq),  cos(mtheta+mq)*cos(malpha), -cos(mtheta+mq)*sin(malpha), -ma*sin(mtheta+mq),
                               0,                 sin(malpha),                 cos(malpha),     md*cos(malpha),
                               0,                           0,                           0,                  1;
    }
    
    mglobal = mbase * mlocal;

    Eigen::Matrix<T, -1, 1> screw;
    T degree;
    mathfunction::SE3ToScrew(mglobal, screw, degree);
    mTwists = mathfunction::adjoint(mlocal) * mpParent.lock()->get_Twists() + screw * mqd;
    mTwistsd = mathfunction::adjoint(mlocal) * mpParent.lock()->get_Twistsd() + mathfunction::LieBracket(mTwists) * screw * mqd + screw * mqdd;

    
}

template<typename T>
void Frame<T>::set_PassiveRefFrame(T _rate, std::shared_ptr<Frame<T>> _refFrame)
{
    if(mframetype != FRAMETYPE::PASSIVE)
    {
        std::stringstream ss;
        ss << "Frame: " << mnid << " is not a passive frame, can not set a passive reference frame\n";
        throw std::invalid_argument(ss.str());
    }
    else
    {
        mPassiveRate = _rate;
        mpRef = _refFrame;
#ifdef DEBUG
        std::cout << "set reference frame for passive frame" << std::endl;
#endif
    }
}

template<typename T>
Eigen::Matrix<T, 4, 4> Frame<T>::get_LocalPose()
{
    return mlocal;
}

template<typename T>
Eigen::Matrix<T, -1, 1> Frame<T>::get_LocalPosOri()
{
    return mathfunction::PoseToPosOri(mlocal);
}

template<typename T>
Eigen::Matrix<T, 3, 1> Frame<T>::get_LocalPos()
{
    return mathfunction::PoseToPos(mlocal);
}

template<typename T>
Eigen::Matrix<T, 4, 4> Frame<T>::get_GlobalPose()
{
    return mglobal;
}

template<typename T>
Eigen::Matrix<T, -1, 1> Frame<T>::get_GlobalPosOri()
{
    return mathfunction::PoseToPosOri(mglobal);
}

template<typename T>
Eigen::Matrix<T, 3, 1> Frame<T>::get_GlobalPos()
{
    return mathfunction::PoseToPos(mglobal);
}

template<typename T>
void Frame<T>::property()
{
#ifdef DEBUG
    std::cout << "+----------------------------------+\n"
              << "id: " << mnid << std::endl
              << mglobal << std::endl;
              
    if(!isRoot())
        std::cout << "parent: " << mpParent.lock()->get_Id() << std::endl;
    else
        std::cout << "parent: NULL\n";
    std::cout << "child: ";
    for(int i=0; i<mvpChildren.size(); i++)
    {
        std::cout << mvpChildren.at(i)->get_Id() << "\t";
    }
    std::cout << "\n+----------------------------------+\n";
#else
    std::cout << "this is the function defined for Debug mode" << std::endl;
#endif
}

template<typename T>
bool Frame<T>::isRoot()
{
    return mpParent.lock() == nullptr;
}

template<typename T>
void Frame<T>::set_Child(std::shared_ptr<Frame<T>> child)
{
    mvpChildren.push_back(child);
}
