#include <robotics/robotarm.hpp>
#include <stdexcept>
#include <robotics/utils.hpp>

int robotarmId = 0;

template<typename T>
RobotArm<T>::RobotArm(std::vector<T> _va, std::vector<T> _valpha, std::vector<T> _vd, std::vector<T> _vtheta, 
                      std::vector<T> _vupperLimit, std::vector<T> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype, 
                      std::vector<FRAMETYPE> _vFrametype, std::vector<Eigen::Matrix<T, 4, 4>> _vTFrame2EE, std::vector<int> _vEEparent)
{
    // insert Frame
    if(_va.size() != _valpha.size() && _va.size() != _vd.size() && _va.size() != _vtheta.size() &&
       _va.size() != _vupperLimit.size() && _va.size() != _vlowerLimit.size() && _va.size() != _vparentId.size() && _va.size() != _vFrametype.size())
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
#ifdef DEBUG
    std::cout << "initialize a robot arm" << std::endl;
    property();
#endif
}

template<typename T>
void RobotArm<T>::set_Angle(Eigen::Matrix<T, -1, 1> _angle)
{
    set_Angle(mathfunction::EigenVectorToStdVector<T>(_angle));
}

template<typename T>
void RobotArm<T>::set_Angle(std::vector<T> _angle)
{
    if(_angle.size() != mnDoF)
    {
        std::stringstream ss;
        ss << "wrong size of angle vector, expect " << mnDoF << "; but given " << _angle.size() << std::endl;
        throw std::invalid_argument(ss.str());
    }

    for(int i=0; i<mvpActiveFrame.size(); i++)
    {
        mvpActiveFrame[i].lock()->set_Angle(_angle[i]);
    }

    for(int i=0; i<mvpPassiveFrame.size(); i++)
    {
        mvpPassiveFrame[i].lock()->set_Angle();
    }

    mpRoot->update();
}

template<typename T>
Eigen::Matrix<T, -1, 1> RobotArm<T>::get_ActiveAngle()
{
    Eigen::Matrix<T, -1, 1> angle(mnDoF);
    for(int i=0; i<mnDoF; i++)
    {
        angle(i) = mvpActiveFrame[i].lock()->get_Angle();
    }
    return angle;
}

template<typename T>
Eigen::Matrix<T, -1, 1> RobotArm<T>::get_PassiveAngle()
{
    Eigen::Matrix<T, -1, 1> angle(mnPassiveDoF);
    for(int i=0; i<mnPassiveDoF; i++)
    {
        angle(i) = mvpPassiveFrame[i].lock()->get_Angle();
    }
    return angle;
}

template<typename T>
Eigen::Matrix<T, -1, 1> RobotArm<T>::get_Angle()
{
    Eigen::Matrix<T, -1, 1> angle(mvpFrame.size());
    for(int i=0; i<mvpFrame.size(); i++)
    {
        angle(i) = mvpFrame[i].lock()->get_Angle();
    }
    return angle;
}

template<typename T>
Eigen::Matrix<T, -1, -1> RobotArm<T>::get_EEPose()
{
    Eigen::Matrix<T, -1, -1> pose(mnEE * 4, 4);
    for(int i=0; i<mnEE; i++)
    {
        pose.block(4*i, 0, 4, 4) = mvpEE[i]->get_GlobalPose();
    }
    return pose;
}

template<typename T>
Eigen::Matrix<T, -1, 1> RobotArm<T>::get_EEPosOri()
{
    Eigen::Matrix<T, -1, 1> PosOri(mnEE * 6);
    for(int i=0; i<mnEE; i++)
    {
        PosOri.block(6*i, 0, 6, 1) = mvpEE[i]->get_GlobalPosOri();
    }
    return PosOri;
} 

template<typename T>
Eigen::Matrix<T, -1, 1> RobotArm<T>::get_EEPos()
{
    Eigen::Matrix<T, -1, 1> Pos(mnEE * 3);
    for(int i=0; i<mnEE; i++)
    {
        Pos.block(3*i, 0, 3, 1) = mvpEE[i]->get_GlobalPos();
    }
    return Pos;
}

template<typename T>
void RobotArm<T>::insert_Frame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype, T _upperLimit, T _lowerLimit, int _parentId)
{
    if(_parentId == -1) 
    {
        std::shared_ptr<Frame<T>> nf(new Frame<T>(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, nullptr));
        int id = nf->get_Id();
        mvpFrame[id] = nf;
        mpRoot = nf;
#ifdef DEBUG
        std::cout << "insert a root\n";
#endif
    }
    else if(_parentId < mvpFrame.size())
    {
        std::shared_ptr<Frame<T>> nf(new Frame<T>(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, mvpFrame[_parentId].lock()));
        int id = nf->get_Id();
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
}

template<typename T>
void RobotArm<T>::insert_endeffector(Eigen::Matrix<T, 4, 4> _TFrame2EE, int _EEparent)
{
    if(_EEparent < mvpFrame.size())
    {
        std::shared_ptr<EndEffector<T>> nee(new EndEffector<T>(mvpFrame[_EEparent].lock(), _TFrame2EE));
        int id = nee->get_Id();
        mvpEE[id] = nee;
    } 
}

template<typename T>
void RobotArm<T>::property()
{
    std::cout << "+----------------------------+" << std::endl;
    std::cout << "robot arm id: " << mnid << std::endl;
    std::cout << "number of Frame: " << mvpFrame.size() << std::endl;
    std::cout << "Active Frame: " << mnDoF << std::endl;
    std::cout << "Passive Frame: " << mnPassiveDoF << std::endl;
    std::cout << "+----------------------------+" << std::endl;
}



