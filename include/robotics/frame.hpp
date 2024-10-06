#ifndef __FRAME_HPP_
#define __FRAME_HPP_

#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

#include <robotics/utils.hpp>

enum DH
{
    TRADITIONAL = 0,
    MODIFIED = 1
};

enum FRAMETYPE
{
    ACTIVE = 0,
    PASSIVE = 1,
    CONSTANT = 2
};


extern int FrameId;
extern int ActiveFrameId;
extern int PassiveFrameId;

template<typename T>
class Frame : public std::enable_shared_from_this<Frame<T>>
{
public:
    /** constructor of frame by using modified dh parameter */
    Frame(T _a, T _alpha, T _d, T _theta, DH _DHtype, FRAMETYPE _frametype,
          T _upperLimit, T _lowerLimit, std::shared_ptr<Frame<T>> _parent);
    
    ~Frame();
    
    /** set angle for passive frame */
    void set_q();
    void set_qd();
    void set_qdd();

    /** set angle*/
    void set_q(T _angle);
    void set_qd(T _qd);
    void set_qdd(T _qdd);

    T get_q(){return mq;}
    T get_qd(){return mqd;}
    T get_qdd(){return mqdd;}

    /** update frame */
    void update();

    void update2();

    void set_PassiveRefFrame(T _rate, std::shared_ptr<Frame<T>> _refFrame);

    /** @brief get local Pose */
    Eigen::Matrix<T, 4, 4> get_LocalPose();

    Eigen::Matrix<T, -1, 1> get_LocalPosOri();

    Eigen::Matrix<T, 3, 1> get_LocalPos();

    /** @brief get global Pose */
    Eigen::Matrix<T, 4, 4> get_GlobalPose();

    Eigen::Matrix<T, -1, 1> get_GlobalPosOri();

    Eigen::Matrix<T, 3, 1> get_GlobalPos();

    void property();

    bool isRoot();

    void set_Child(std::shared_ptr<Frame<T>> child);

    int get_Id(){return mnid;}

    int get_JointId(){return mnJointId;}

    FRAMETYPE get_FrameType(){return mframetype;}

    /** getter for DH parameter*/
    T get_a(){return ma;}
    T get_d(){return md;}
    T get_alpha(){return malpha;}
    T get_theta(){return mtheta;}

    Eigen::Matrix<T, -1, 1> get_Twists(){return mTwists;}

    Eigen::Matrix<T, -1, 1> get_Twistsd(){return mTwistsd;}



protected:
    Eigen::Matrix<T, 4, 4> mlocal;
    Eigen::Matrix<T, 4, 4> mglobal;
    Eigen::Matrix<T, 4, 4> mbase;

    /** @brief DH type */
    DH mDHtype;

    /** @brief frame type */
    FRAMETYPE mframetype;

    /** @brief reference frame */
    std::weak_ptr<Frame<T>> mpRef;
    float mPassiveRate;
    
    /** @brief DH parameter */
    const T ma;
    const T malpha;
    const T md;
    const T mtheta;

    /** @brief current angle */
    T mq{0};

    T mqd{0};

    T mqdd{0};

    Eigen::Matrix<T, -1, 1> mTwists;

    Eigen::Matrix<T, -1, 1> mTwistsd;

    /** @brief joint limit */
    T mupperLimit;
    T mlowerLimit;

    /** @brief id of frame */
    int mnid;
    int mnJointId;

    /** @brief pointer to children*/
    std::vector<std::shared_ptr<Frame<T>>> mvpChildren;

    /** @brief pointer to parent */
    std::weak_ptr<Frame<T>> mpParent;
};

template class Frame<float>;
template class Frame<double>;

typedef Frame<float> Framef;
typedef Frame<double> Framed;

#endif // __FRAME_HPP_