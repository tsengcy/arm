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

    for(int i=0; i<RobotArm::mvpFrame.size(); i++)
    {
        if(mvpFrame[i].lock()->isLeaf())
        {
            mvpLeaf.push_back(mvpFrame[i].lock());
        }
    }

    RobotArm::setGravity(true);
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
        std::cout << id << std::endl;
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

void dynRobotArm::Inverse_Dynamic(Eigen::VectorXf _q, Eigen::VectorXf _qd, Eigen::VectorXf _qdd, std::vector<Eigen::VectorXf> _externalForce, std::vector<Eigen::Vector3f> _externalForcePos, std::vector<int> _externalForceId)
{
    if(_externalForce.size() != _externalForceId.size() || _externalForce.size() != _externalForcePos.size())
    {
        throw std::invalid_argument("wrong external force size");
    }
    // update configure
    RobotArm::set_q(_q);
    RobotArm::set_qd(_qd);
    RobotArm::set_qdd(_qdd);

    for(int i=0; i<_externalForce.size(); i++)
    {
        if(_externalForceId[i] >= mvpFrame.size())
        {
            throw std::invalid_argument("external force Id error");
        }
            std::shared_ptr<dynFrame> ptr = std::dynamic_pointer_cast<dynFrame>(mvpFrame[_externalForceId[i]].lock());
            ptr->addExternalForce(_externalForce[i], _externalForcePos[i]);
    }

    

    // forward
    mpRoot->update2();

    // backward
    std::dynamic_pointer_cast<dynFrame>(mpRoot)->set_Backward();

    std::queue<std::shared_ptr<dynFrame>> mqDynFrame;
    for(int i=0; i<mvpLeaf.size(); i++)
    {
        mqDynFrame.push(std::dynamic_pointer_cast<dynFrame>(mvpLeaf[i].lock()));
    }
    while(!mqDynFrame.empty())
    {
        std::shared_ptr<dynFrame> ptr = mqDynFrame.front();
        mqDynFrame.pop();

        ptr->InverseDynamicBackward();

        ptr = std::dynamic_pointer_cast<dynFrame>(ptr->get_Parent());
        if(ptr != nullptr && ptr->check_Backward())
        {
            mqDynFrame.push(ptr);
        }
    }
}

Eigen::VectorXf dynRobotArm::get_Torque()
{
    Eigen::VectorXf torque = Eigen::VectorXf::Zero(mnDoF);
    for(int i=0; i<mvpActiveFrame.size(); i++)
    {
        std::shared_ptr<dynFrame> ptr = std::dynamic_pointer_cast<dynFrame>(mvpActiveFrame[i].lock());
        int id = ptr->get_JointId();
        torque(id) = ptr->get_Torque();
    }

    for(int i=0; i<mvpPassiveFrame.size(); i++)
    {
        std::shared_ptr<dynFrame> ptr = std::dynamic_pointer_cast<dynFrame>(mvpPassiveFrame[i].lock());
        int id = ptr->get_RefId();
        torque(id) = ptr->get_Torque();
    }

    return torque;
}

void dynRobotArm::Forward_Dynamic(Eigen::VectorXf _q, Eigen::VectorXf _qd, Eigen::VectorXf _jointTorque,
                                  std::vector<Eigen::VectorXf> _externalforce, std::vector<Eigen::Vector3f> _externalforcePos, std::vector<int> _externalforceId)
{
    if(_externalforce.size() != _externalforceId.size() || _externalforce.size() != _externalforcePos.size())
    {
        throw std::invalid_argument("wrong external force data size");
    }
    RobotArm::setGravity(true);
    Eigen::VectorXf qdd = Eigen::VectorXf::Zero(mnDoF);

    Inverse_Dynamic(_q, _qd, qdd);
    Eigen::VectorXf h = get_Torque();

    // std::cout << "h\n" << h << "\n----\n";

    Eigen::MatrixXf M(mnDoF, mnDoF);
    RobotArm::setGravity(false);
    
    for(int i=0; i<mnDoF; i++)
    {
        qdd(i) = 1;
        Inverse_Dynamic(_q, Eigen::VectorXf::Zero(mnDoF), qdd);
        M.block(0, i, mnDoF, 1) = get_Torque();
        qdd(i) = 0;
    }

    Eigen::VectorXf RHS = _jointTorque - h;

    for(int i=0; i<_externalforce.size(); i++)
    {
        int id = _externalforceId[i];
        Eigen::Matrix4f Mat = mvpFrame[id].lock()->get_GlobalPose();
        Eigen::Vector3f pos = Mat.block<3,3>(0,0) * _externalforcePos[i] + Mat.block<3,1>(0,3);
        
        Eigen::MatrixXf J = Eigen::MatrixXf::Zero(6, mnDoF);
        mvpFrame[id].lock()->get_JacobainPosOri(J, pos);

        _externalforce[i].block<3,1>(0,0) = Mat.block<3,3>(0,0) * _externalforce[i].block<3,1>(0,0);
        _externalforce[i].block<3,1>(3,0) = Mat.block<3,3>(0,0) * _externalforce[i].block<3,1>(3,0);

        RHS += J.transpose() * _externalforce[i];
    }
    qdd = M.inverse() * RHS;

    set_qdd(qdd);
    RobotArm::setGravity(true);
}

