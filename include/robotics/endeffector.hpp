#ifndef __ENDEFFECTOR_HPP_
#define __ENDEFFECTOR_HPP_

#include <iostream>
#include <memory>
#include "robotics/frame.hpp"
#include "robotics/utils.hpp"

extern int EEId;

class EndEffector
{
public:
    EndEffector(std::shared_ptr<Frame> frame, Eigen::Matrix4f trans);

    Eigen::Matrix4f get_GlobalPose();

    Eigen::VectorXf get_GlobalPosOri();

    Eigen::Vector3f get_GlobalPos();

    int get_Id(){return mnid;}

private:
    void update();

private:
    std::weak_ptr<Frame> mpFrame;
    
    Eigen::Matrix4f mTFrame2EE;

    Eigen::Matrix4f mTGlobal;

    int mnid;
};

#endif // __ENDEFFECTOR_HPP_