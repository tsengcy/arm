#include <robotics/dynrobotarm.hpp>
#include <stdexcept>

dynRobotArm::dynRobotArm(std::vector<float> _va, std::vector<float> _valpha, std::vector<float> _vd, std::vector<float> _vtheta, 
                         std::vector<float> _vupperLimit, std::vector<float> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype,
                         std::vector<FRAMETYPE> _vFrametype, std::vector<float> _vmass, std::vector<Eigen::Matrix3f> _vinertira, std::vector<Eigen::Matrix4f> _vTCOM,
                         std::vector<Eigen::Matrix4f> _vTFrame2EE, std::vector<int> _vEEparent)
                         :RobotArm()
{
    if(_va.size() != _valpha.size() || _va.size() != _vd.size() || _va.size() != _vtheta.size() ||
       _va.size() != _vupperLimit.size() || _va.size() != _vlowerLimit.size() || 
       _va.size() != _vparentId.size() || _va.size() != _vFrametype.size() ||
       _va.size() != _vmass.size() || _va.size() != _vinertira.size() || _va.size() != _vTCOM.size())
    {
        throw std::invalid_argument("wrong size of frame parameter\n");
    }

    RobotArm::mvpFrame.resize(_va.size());

    for(int i=0; i<_va.size(); i++)
    {
        insert_DynFrame(_va[i], _valpha[i], _vd[i], _vtheta[i], _DHtype, _vFrametype[i], _vupperLimit[i], _vlowerLimit[i], _vparentId[i],
                        _vmass[i], _vinertira[i], _vTCOM[i]);
    }

    RobotArm::mvpActiveFrame.resize(RobotArm::mnDoF);
    RobotArm::mvpPassiveFrame.resize(RobotArm::mnPassiveDoF);
    for(int i=0; i<RobotArm::mvpFrame.size(); i++)
    {
        int id = RobotArm::mvpFrame[i].lock()->get_JointId();
        if(RobotArm::mvpFrame[i].lock()->get_FrameType() == FRAMETYPE::ACTIVE)
        {
            RobotArm::mvpActiveFrame[id] = RobotArm::mvpFrame[i];
        }
        else if(RobotArm::mvpFrame[i].lock()->get_FrameType() == FRAMETYPE::PASSIVE)
        {
            RobotArm::mvpPassiveFrame[id] = RobotArm::mvpFrame[i];
        }
    }

    RobotArm::mpRoot->update();

    // insert end effector 
    if(_vTFrame2EE.size() != _vEEparent.size())
    {
        throw std::invalid_argument("wrong size of EE parameter");
    }

    RobotArm::mvpEE.resize(_vTFrame2EE.size());

    for(int i=0; i<_vTFrame2EE.size(); i++)
    {
        insert_endeffector(_vTFrame2EE[i], _vEEparent[i]);
        RobotArm::mnEE++;
    }
}

void dynRobotArm::insert_DynFrame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype,
                                  float _upperLimit, float _lowerLimit, int _parentId,
                                  float _mass, Eigen::Matrix3f _inertira, Eigen::Matrix4f _TCOM)
{
    int id;
    if(_parentId == -1) 
    {
        std::shared_ptr<dynFrame> nf(new dynFrame(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, nullptr, _mass, _inertira, _TCOM));
        id = nf->get_Id();
        RobotArm::mvpFrame[id] = nf;
        RobotArm::mpRoot = nf;
#ifdef DEBUG
        std::cout << "insert a root\n";
#endif
    }
    else if(_parentId < mvpFrame.size())
    {
        std::shared_ptr<dynFrame> nf(new dynFrame(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, mvpFrame[_parentId].lock(), _mass, _inertira, _TCOM));
        id = nf->get_Id();
        RobotArm::mvpFrame[id] = nf;
        RobotArm::mvpFrame[_parentId].lock()->set_Child(nf);
#ifdef DEBUG
        std::cout << "insert frame id: " << nf->get_Id() << std::endl;
#endif
    }
    else
    {
        throw std::invalid_argument("parent id is out of range");
    }

    if(_frametype == FRAMETYPE::ACTIVE)
        RobotArm::mnDoF++;
    else if(_frametype == FRAMETYPE::PASSIVE)
        RobotArm::mnPassiveDoF++;
}

void dynRobotArm::Inverse_Dynamic(Eigen::VectorXf _q, Eigen::VectorXf _qd, Eigen::VectorXf _qdd)
{
    // update configure
    RobotArm::set_q(_q);
    RobotArm::set_qd(_qd);
    RobotArm::set_qdd(_qdd);

    // forward
    std::queue<std::shared_ptr<Frame>> mqforward;
    mqforward.push(mpRoot);

    while(mqforward.empty())
    {
        std::shared_ptr<Frame> pDynFrame = mqforward.front();
    }

    // backward

}



