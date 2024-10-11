#include <robotics/robotarm.hpp>
#include <stdexcept>
#include <robotics/utils.hpp>

int robotarmId = 0;

RobotArm::RobotArm()
{
    mnid = robotarmId++;
}

RobotArm::RobotArm(std::vector<float> _va, std::vector<float> _valpha, std::vector<float> _vd, std::vector<float> _vtheta, 
                   std::vector<float> _vupperLimit, std::vector<float> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype, 
                   std::vector<FRAMETYPE> _vFrametype, std::vector<Eigen::Matrix4f> _vTFrame2EE, std::vector<int> _vEEparent)
{
    // insert Frame
    if(_va.size() != _valpha.size() || _va.size() != _vd.size() || _va.size() != _vtheta.size() ||
       _va.size() != _vupperLimit.size() || _va.size() != _vlowerLimit.size() || 
       _va.size() != _vparentId.size() || _va.size() != _vFrametype.size())
    {
        throw std::invalid_argument("wrong size of frame parameter\n");
    }

    mvpFrame.resize(_va.size());

    for(int i=0; i<_va.size(); i++)
    {
        insert_Frame(_va[i], _valpha[i], _vd[i], _vtheta[i], _DHtype, _vFrametype[i], _vupperLimit[i], _vlowerLimit[i], _vparentId[i]);
    }

    mvpActiveFrame.resize(mnDoF);
    mvpPassiveFrame.resize(mnPassiveDoF);

    for(int i=0; i<mvpFrame.size(); i++)
    {
        int id = mvpFrame[i].lock()->get_JointId();
        if(mvpFrame[i].lock()->get_FrameType() == FRAMETYPE::ACTIVE)
        {
            mvpActiveFrame[id] = mvpFrame[i];
        }
        else if(mvpFrame[i].lock()->get_FrameType() == FRAMETYPE::PASSIVE)
        {
            mvpPassiveFrame[id] = mvpFrame[i];
        }
    }

    mpRoot->update();

    // insert end effector 
    if(_vTFrame2EE.size() != _vEEparent.size())
    {
        throw std::invalid_argument("wrong size of EE parameter");
    }

    mvpEE.resize(_vTFrame2EE.size());

    for(int i=0; i<_vTFrame2EE.size(); i++)
    {
        insert_endeffector(_vTFrame2EE[i], _vEEparent[i]);
        mnEE++;
    }

    mnid = robotarmId++;

    for(int i=0; i<mvpFrame.size(); i++)
    {
        if(mvpFrame[i].lock()->isLeaf())
        {
            mvpLeaf.push_back(mvpFrame[i].lock());
        }
    }
#ifdef DEBUG
    std::cout << "initialize a robot arm" << std::endl;
    property();
#endif
}

void RobotArm::set_q(Eigen::VectorXf& _q)
{
    // set_q(mathfunction::EigenVectorToStdVector(_q));
    if(_q.rows() != mnDoF)
    {
        std::stringstream ss;
        ss << "wrong size of angle vector, expect " << mnDoF << "; but given " << _q.size() << std::endl;
        throw std::invalid_argument(ss.str());
    }

    for(int i=0; i<mvpActiveFrame.size(); i++)
    {
        mvpActiveFrame[i].lock()->set_q(_q(i));
    }

    for(int i=0; i<mvpPassiveFrame.size(); i++)
    {
        mvpPassiveFrame[i].lock()->set_q();
    }

    mpRoot->update();
}

void RobotArm::set_q(std::vector<float> _q)
{
    if(_q.size() != mnDoF)
    {
        std::stringstream ss;
        ss << "wrong size of angle vector, expect " << mnDoF << "; but given " << _q.size() << std::endl;
        throw std::invalid_argument(ss.str());
    }

    for(int i=0; i<mvpActiveFrame.size(); i++)
    {
        mvpActiveFrame[i].lock()->set_q(_q[i]);
    }

    for(int i=0; i<mvpPassiveFrame.size(); i++)
    {
        mvpPassiveFrame[i].lock()->set_q();
    }

    mpRoot->update();
}

void RobotArm::set_qd(Eigen::VectorXf _qd)
{
    set_qd(mathfunction::EigenVectorToStdVector(_qd));
}

void RobotArm::set_qd(std::vector<float> _qd)
{
    if(_qd.size() != mnDoF)
    {
        std::stringstream ss;
        ss << "wrong size of angle vector, expect " << mnDoF << "; but given " << _qd.size() << std::endl;
        throw std::invalid_argument(ss.str());
    }

    for(int i=0; i<mvpActiveFrame.size(); i++)
    {
        mvpActiveFrame[i].lock()->set_qd(_qd[i]);
    }

    for(int i=0; i<mvpPassiveFrame.size(); i++)
    {
        mvpPassiveFrame[i].lock()->set_qd();
    }
}

void RobotArm::set_qdd(Eigen::VectorXf _qdd)
{
    set_qdd(mathfunction::EigenVectorToStdVector(_qdd));
}

void RobotArm::set_qdd(std::vector<float> _qdd)
{
    if(_qdd.size() != mnDoF)
    {
        std::stringstream ss;
        ss << "wrong size of angle vector, expect " << mnDoF << "; but given " << _qdd.size() << std::endl;
        throw std::invalid_argument(ss.str());
    }

    for(int i=0; i<mvpActiveFrame.size(); i++)
    {
        mvpActiveFrame[i].lock()->set_qdd(_qdd[i]);
    }

    for(int i=0; i<mvpPassiveFrame.size(); i++)
    {
        mvpPassiveFrame[i].lock()->set_qdd();
    }
}

Eigen::VectorXf RobotArm::get_Activeq()
{
    Eigen::VectorXf angle(mnDoF);
    for(int i=0; i<mnDoF; i++)
    {
        angle(i) = mvpActiveFrame[i].lock()->get_q();
    }
    return angle;
}

Eigen::VectorXf RobotArm::get_Passiveq()
{
    Eigen::VectorXf angle(mnPassiveDoF);
    for(int i=0; i<mnPassiveDoF; i++)
    {
        angle(i) = mvpPassiveFrame[i].lock()->get_q();
    }
    return angle;
}

Eigen::VectorXf RobotArm::get_q()
{
    Eigen::VectorXf angle(mvpFrame.size());
    for(int i=0; i<mvpFrame.size(); i++)
    {
        angle(i) = mvpFrame[i].lock()->get_q();
    }
    return angle;
}

Eigen::MatrixXf RobotArm::get_EEPose()
{
    Eigen::MatrixXf pose(mnEE * 4, 4);
    for(int i=0; i<mnEE; i++)
    {
        pose.block(4*i, 0, 4, 4) = mvpEE[i]->get_GlobalPose();
    }
    return pose;
}

Eigen::VectorXf RobotArm::get_EEPosOri()
{
    Eigen::VectorXf PosOri(mnEE * 6);
    for(int i=0; i<mnEE; i++)
    {
        PosOri.block(6*i, 0, 6, 1) = mvpEE[i]->get_GlobalPosOri();
    }
    return PosOri;
} 

Eigen::VectorXf RobotArm::get_EEPos()
{
    Eigen::VectorXf Pos(mnEE * 3);
    for(int i=0; i<mnEE; i++)
    {
        Pos.block(3*i, 0, 3, 1) = mvpEE[i]->get_GlobalPos();
    }
    return Pos;
}

Eigen::VectorXf RobotArm::get_Twist()
{
    Eigen::VectorXf twist(mnEE * 6);
    for(int i=0; i<mnEE; i++)
    {
        twist.block(6*i, 0, 6, 1) = mvpEE[i]->get_Twist();
    }
    return twist;
}

Eigen::VectorXf RobotArm::get_Twistd()
{
    Eigen::VectorXf twistd(mnEE * 6);
    for(int i=0; i<mnEE; i++)
    {
        twistd.block(6*i, 0, 6, 1) = mvpEE[i]->get_Twistd();
    }
    return twistd;
}

Eigen::VectorXf RobotArm::get_GlobalTwist()
{
    Eigen::VectorXf twist(mnEE * 6);
    for(int i=0; i<mnEE; i++)
    {
        twist.block(6*i, 0, 6, 1) = mvpEE[i]->get_GlobalTwist();
    }
    return twist;
}

Eigen::VectorXf RobotArm::get_GlobalTwistd()
{
    Eigen::VectorXf twistd(mnEE * 6);
    for(int i=0; i<mnEE; i++)
    {
        twistd.block(6*i, 0, 6, 1) = mvpEE[i]->get_GlobalTwistd();
    }
    return twistd;
}

int RobotArm::insert_Frame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype, float _upperLimit, float _lowerLimit, int _parentId)
{
    int id;
    if(_parentId == -1) 
    {
        std::shared_ptr<Frame> nf(new Frame(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, nullptr));
        id = nf->get_Id();
        mvpFrame[id] = nf;
        mpRoot = nf;
#ifdef DEBUG
        std::cout << "insert a root\n";
#endif
    }
    else if(_parentId < mvpFrame.size())
    {
        std::shared_ptr<Frame> nf(new Frame(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, mvpFrame[_parentId].lock()));
        id = nf->get_Id();
        mvpFrame[id] = nf;
        mvpFrame[_parentId].lock()->set_Child(nf);
#ifdef DEBUG
        std::cout << "insert frame id: " << nf->get_Id() << std::endl;
#endif
    }
    else
    {
        throw std::invalid_argument("parent id is out of range");
    }

    if(_frametype == FRAMETYPE::ACTIVE)
        mnDoF++;
    else if(_frametype == FRAMETYPE::PASSIVE)
        mnPassiveDoF++;

    return id;
}

void RobotArm::insert_endeffector(Eigen::Matrix4f _TFrame2EE, int _EEparent)
{
    if(_EEparent < mvpFrame.size())
    {
        std::shared_ptr<EndEffector> nee(new EndEffector(mvpFrame[_EEparent].lock(), _TFrame2EE));
        int id = nee->get_Id();
        mvpEE[id] = nee;
    } 
}

void RobotArm::property()
{
    std::cout << "+----------------------------+" << std::endl;
    std::cout << "robot arm id: " << mnid << std::endl;
    std::cout << "number of Frame: " << mvpFrame.size() << std::endl;
    std::cout << "Active Frame: " << mnDoF << std::endl;
    std::cout << "Passive Frame: " << mnPassiveDoF << std::endl;
    std::cout << "+----------------------------+" << std::endl;
}

Eigen::MatrixXf RobotArm::get_JacobainPos()
{
    Eigen::MatrixXf J = Eigen::MatrixXf::Zero(3*mnEE, mnDoF);

    for(int i=0; i<mnEE; i++)
    {
        Eigen::Vector3f eePos = mvpEE[i]->get_GlobalPos();
        for(int j=0; j<mnDoF; j++)
        {
            J.block<3,1>(3*i,j) = mvpActiveFrame[j].lock()->get_JacobainPos(eePos);
        }

        for(int j=0; j<mvpPassiveFrame.size(); j++)
        {
            int refId = mvpPassiveFrame[j].lock()->get_RefFrameId();
            J.block<3,1>(3*i, refId) += mvpPassiveFrame[j].lock()->get_JacobainPos(eePos);
        }
    }

    return J;
}

Eigen::MatrixXf RobotArm::get_JacobainPosOri()
{
    Eigen::MatrixXf J = Eigen::MatrixXf::Zero(6*mnEE, mnDoF);

    for(int i=0; i<mnEE; i++)
    {
        Eigen::Vector3f eePos = mvpEE[i]->get_GlobalPos();
        for(int j=0; j<mnDoF; j++)
        {
            J.block<6,1>(6*i,j) = mvpActiveFrame[j].lock()->get_JacobainPosOri(eePos);
        }

        for(int j=0; j<mvpPassiveFrame.size(); j++)
        {
            int refId = mvpPassiveFrame[j].lock()->get_RefFrameId();
            J.block<6,1>(6*i, refId) += mvpPassiveFrame[j].lock()->get_JacobainPosOri(eePos);
        }
    }

    return J;
}

#define IK_MAXIMUM_ITERATION 1000
#define IK_POS_ERROR 0.001
#define IK_WEIGHT 0.001

Eigen::VectorXf RobotArm::IK_Pos(Eigen::VectorXf _eePos)
{
    //check the size of ee
    if(_eePos.rows() != mnEE * 3)
    {
        throw std::invalid_argument("the given end effector pos is wrong");
    }

    Eigen::VectorXf error = _eePos - get_EEPos();
    Eigen::VectorXf angle = get_Activeq();
    int iter = 0;
    while(error.norm() > IK_POS_ERROR && iter < IK_MAXIMUM_ITERATION)
    {
        Eigen::MatrixXf J = get_JacobainPos();
        Eigen::VectorXf dq = mathfunction::pseudoInverse(J) * error;

        angle = angle + dq;

        for(int i=0; i<mvpActiveFrame.size(); i++)
        {
            mvpActiveFrame[i].lock()->checkq(angle(i));
        }
        set_q(angle);
        error = _eePos - get_EEPos();
        iter++;
    }
#ifdef DEBUG
        std::cout << "solve IK with " << iter+1 << " iteration.\n";
        std::cout << "error: " << error.norm() << std::endl;
#endif

    return angle;
} 

Eigen::VectorXf RobotArm::IK_PosOri(Eigen::VectorXf _eePosOri)
{
    if(_eePosOri.rows() != mnEE * 6)
    {
        throw std::invalid_argument("the given end effector pos is wrong");
    }

    Eigen::VectorXf error = _eePosOri - get_EEPosOri();
    Eigen::VectorXf angle = get_Activeq();
    int iter = 0;
    while(error.norm() > IK_POS_ERROR && iter < IK_MAXIMUM_ITERATION)
    {
        Eigen::MatrixXf J = get_JacobainPosOri();
        Eigen::VectorXf dq = mathfunction::pseudoInverse(J) * error;

        angle = angle + dq;

        for(int i=0; i<mvpActiveFrame.size(); i++)
        {
            mvpActiveFrame[i].lock()->checkq(angle(i));
        }
        set_q(angle);
        error = _eePosOri - get_EEPosOri();
        iter++;
    }
#ifdef DEBUG
        std::cout << "solve IK with " << iter+1 << " iteration.\n";
        std::cout << "error: " << error.norm() << std::endl;
#endif

    return angle;
} 
