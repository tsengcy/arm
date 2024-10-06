#include <robotics/dynrobotarm.hpp>
#include <stdexcept>

template<typename T>
dynRobotArm<T>::dynRobotArm(std::vector<T> _va, std::vector<T> _valpha, std::vector<T> _vd, std::vector<T> _vtheta, 
                            std::vector<T> _vupperLimit, std::vector<T> _vlowerLimit, std::vector<int> _vparentId, DH _DHtype,
                            std::vector<FRAMETYPE> _vFrametype, std::vector<T> _vmass, std::vector<Eigen::Matrix<T, 3, 3>> _vinertira, std::vector<Eigen::Matrix<T, 4, 4>> _vTCOM,
                            std::vector<Eigen::Matrix<T, 4, 4>> _vTFrame2EE, std::vector<int> _vEEparent)
                            :RobotArm<T>()
{
    if(_va.size() != _valpha.size() || _va.size() != _vd.size() || _va.size() != _vtheta.size() ||
       _va.size() != _vupperLimit.size() || _va.size() != _vlowerLimit.size() || 
       _va.size() != _vparentId.size() || _va.size() != _vFrametype.size() ||
       _va.size() != _vmass.size() || _va.size() != _vinertira.size() || _va.size() != _vTCOM.size())
    {
        throw std::invalid_argument("wrong size of frame parameter\n");
    }

    RobotArm<T>::mvpFrame.resize(_va.size());
    mvpDynFrame.resize(_va.size());

    for(int i=0; i<_va.size(); i++)
    {
        insert_DynFrame(_va[i], _valpha[i], _vd[i], _vtheta[i], _DHtype, _vFrametype[i], _vupperLimit[i], _vlowerLimit[i], _vparentId[i],
                        _vmass[i], _vinertira[i], _vTCOM[i]);
    }

    RobotArm<T>::mvpActiveFrame.resize(RobotArm<T>::mnDoF);
    RobotArm<T>::mvpPassiveFrame.resize(RobotArm<T>::mnPassiveDoF);
    for(int i=0; i<RobotArm<T>::mvpFrame.size(); i++)
    {
        int id = RobotArm<T>::mvpFrame[i].lock()->get_JointId();
        if(RobotArm<T>::mvpFrame[i].lock()->get_FrameType() == FRAMETYPE::ACTIVE)
        {
            RobotArm<T>::mvpActiveFrame[id] = RobotArm<T>::mvpFrame[i];
        }
        else if(RobotArm<T>::mvpFrame[i].lock()->get_FrameType() == FRAMETYPE::PASSIVE)
        {
            RobotArm<T>::mvpPassiveFrame[id] = RobotArm<T>::mvpFrame[i];
        }
    }

    RobotArm<T>::mpRoot->update();

    // insert end effector 
    if(_vTFrame2EE.size() != _vEEparent.size())
    {
        throw std::invalid_argument("wrong size of EE parameter");
    }

    RobotArm<T>::mvpEE.resize(_vTFrame2EE.size());

    for(int i=0; i<_vTFrame2EE.size(); i++)
    {
        insert_endeffector(_vTFrame2EE[i], _vEEparent[i]);
        RobotArm<T>::mnEE++;
    }
}

template<typename T>
void dynRobotArm<T>::insert_DynFrame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype, T _upperLimit, T _lowerLimit, int _parentId,
                         T _mass, Eigen::Matrix<T, 3, 3> _inertira, Eigen::Matrix<T, 4, 4> _TCOM)
{
    int id;
    if(_parentId == -1) 
    {
        std::shared_ptr<dynFrame<T>> nf(new dynFrame<T>(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, nullptr, _mass, _inertira, _TCOM));
        id = nf->get_Id();
        RobotArm<T>::mvpFrame[id] = nf;
        RobotArm<T>::mpRoot = nf;
#ifdef DEBUG
        std::cout << "insert a root\n";
#endif
    }
    else if(_parentId < mvpFrame.size())
    {
        std::shared_ptr<dynFrame<T>> nf(new dynFrame<T>(_a, _alpha, _d, _theta, _DHtype, _frametype, _upperLimit, _lowerLimit, mvpFrame[_parentId].lock(), _mass, _inertira, _TCOM));
        id = nf->get_Id();
        RobotArm<T>::mvpFrame[id] = nf;
        RobotArm<T>::mvpFrame[_parentId].lock()->set_Child(nf);
#ifdef DEBUG
        std::cout << "insert frame id: " << nf->get_Id() << std::endl;
#endif
    }
    else
    {
        throw std::invalid_argument("parent id is out of range");
    }

    if(_frametype == FRAMETYPE::ACTIVE)
        RobotArm<T>:mnDoF++;
    else if(_frametype == FRAMETYPE::PASSIVE)
        RobotArm<T>:mnPassiveDoF++;
}

template<typename T>
void dynRobotArm<T>::Inverse_Dynamic(Eigen::Matrix<T, -1, 1> _q, Eigen::Matrix<T, -1, 1> _qd, Eigen::Matrix<T, -1, 1> _qdd)
{
    // update configure
    RobotArm<T>::set_q(_q);
    RobotArm<T>::set_qd(_qd);
    RobotArm<T>::set_qdd(_qdd);

    // forward
    std::queue<std::weak_ptr<dynFrame<T>>> mqforward;
    mqforward.push(mDynRoot);

    while(mqforward.empty())
    {
        std::shared_ptr<dynFrame<T>> pDynFrame = mqforward.pop();
    }

    // backward

}



