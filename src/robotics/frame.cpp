#include <robotics/frame.hpp>

int FrameId = 0;
int ActiveFrameId = 0;
int PassiveFrameId = 0;

Frame::Frame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype, 
                float _upperLimit, float _lowerLimit, std::shared_ptr<Frame> _parent) 
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
        mbase = Eigen::Matrix4f::Identity();
    }
    mq = 0;
}

Frame::~Frame()
{
    std::cout << "reset id: " << mnid << std::endl;
    for(auto &ptr : mvpChildren)
    {
        ptr.reset();
    }
}

void Frame::set_q(float _angle)
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

void Frame::set_qd(float _qd)
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

void Frame::set_qdd(float _qdd)
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

void Frame::set_q()
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

void Frame::set_qd()
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

void Frame::set_qdd()
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

void Frame::update()
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

void Frame::update2()
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

    Eigen::VectorXf screw;
    float degree;
    mathfunction::SE3ToScrew(mglobal, screw, degree);
    mTwists = mathfunction::adjoint(mlocal) * mpParent.lock()->get_Twists() + screw * mqd;
    mTwistsd = mathfunction::adjoint(mlocal) * mpParent.lock()->get_Twistsd() + mathfunction::LieBracket(mTwists) * screw * mqd + screw * mqdd;

    
}

void Frame::set_PassiveRefFrame(float _rate, std::shared_ptr<Frame> _refFrame)
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

Eigen::Matrix4f Frame::get_LocalPose()
{
    return mlocal;
}

Eigen::VectorXf Frame::get_LocalPosOri()
{
    return mathfunction::PoseToPosOri(mlocal);
}

Eigen::Vector3f Frame::get_LocalPos()
{
    return mathfunction::PoseToPos(mlocal);
}

Eigen::Matrix4f Frame::get_GlobalPose()
{
    return mglobal;
}

Eigen::VectorXf Frame::get_GlobalPosOri()
{
    return mathfunction::PoseToPosOri(mglobal);
}

Eigen::Vector3f Frame::get_GlobalPos()
{
    return mathfunction::PoseToPos(mglobal);
}

void Frame::property()
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

bool Frame::isRoot()
{
    return mpParent.lock() == nullptr;
}

void Frame::set_Child(std::shared_ptr<Frame> child)
{
    mvpChildren.push_back(child);
}
