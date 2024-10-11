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

class Frame : public std::enable_shared_from_this<Frame>
{
public:
    /** constructor of frame by using modified dh parameter */
    Frame(float _a, float _alpha, float _d, float _theta, DH _DHtype, FRAMETYPE _frametype,
          float _upperLimit, float _lowerLimit, std::shared_ptr<Frame> _parent);
    
    virtual ~Frame();
    
    /** set angle for passive frame */
    void set_q();
    void set_qd();
    void set_qdd();

    /** set angle*/
    void set_q(float _angle);
    void set_qd(float _qd);
    void set_qdd(float _qdd);

    float get_q(){return mq;}
    float get_qd(){return mqd;}
    float get_qdd(){return mqdd;}

    /** update frame */
    void update();

    void update2();

    void set_PassiveRefFrame(float _rate, std::shared_ptr<Frame> _refFrame);

    /** @brief get local Pose */
    Eigen::Matrix4f get_LocalPose();

    Eigen::VectorXf get_LocalPosOri();

    Eigen::Vector3f get_LocalPos();

    /** @brief get global Pose */
    Eigen::Matrix4f get_GlobalPose();

    Eigen::VectorXf get_GlobalPosOri();

    Eigen::Vector3f get_GlobalPos();

    void property();

    bool isRoot();

    void set_Child(std::shared_ptr<Frame> child);

    int get_Id(){return mnid;}

    int get_JointId(){return mnJointId;}

    int get_RefId();

    FRAMETYPE get_FrameType(){return mframetype;}

    int get_RefFrameId();

    bool isLeaf(){return mvpChildren.size() == 0;}

    /** getter for DH parameter*/
    float get_a(){return ma;}
    float get_d(){return md;}
    float get_alpha(){return malpha;}
    float get_theta(){return mtheta;}

    Eigen::VectorXf get_Twist(){return mTwist;}

    Eigen::VectorXf get_Twistd(){return mTwistd;}

    Eigen::VectorXf get_Twistgd(){return mTwistgd;}

    std::shared_ptr<Frame> get_Parent(){return mpParent.lock();}
    
    void get_JacobainPosOri(Eigen::MatrixXf& J, Eigen::Vector3f _pos);

    void get_JacobainPos(Eigen::MatrixXf& J, Eigen::Vector3f _pos);

    void checkq(float& _q);

    void setGravity(bool _useGravity);

    Eigen::VectorXf getGravity(){return GRAVTIY;}

protected:
    Eigen::Matrix4f mlocal;
    Eigen::Matrix4f mglobal;
    Eigen::Matrix4f mbase;

    /** @brief DH type */
    DH mDHtype;

    /** @brief frame type */
    FRAMETYPE mframetype;

    /** @brief reference frame */
    std::weak_ptr<Frame> mpRef;
    float mPassiveRate{1};
    
    /** @brief DH parameter */
    const float ma;
    const float malpha;
    const float md;
    const float mtheta;

    /** @brief current angle */
    float mq{0};

    float mqd{0};

    float mqdd{0};

    Eigen::VectorXf mTwist{Eigen::VectorXf::Zero(6)};

    Eigen::VectorXf mTwistd{Eigen::VectorXf::Zero(6)};

    Eigen::VectorXf mTwistgd{Eigen::VectorXf::Zero(6)};

    /** @brief joint limit */
    float mupperLimit;
    float mlowerLimit;

    /** @brief id of frame */
    int mnid;
    int mnJointId;

    /** @brief pointer to children*/
    std::vector<std::shared_ptr<Frame>> mvpChildren;

    /** @brief pointer to parent */
    std::weak_ptr<Frame> mpParent;

    bool mbuseGravity;

    Eigen::VectorXf GRAVTIY{Eigen::VectorXf::Zero(6)};
};

#endif // __FRAME_HPP_