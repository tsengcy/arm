#ifndef __ROBOT_HPP_
#define __ROBOT_HPP_

#include <iostream>
#include <memory>
#include "robotics/frame.hpp"

class robot
{
public:
    robot();
    
private:
    std::shared_ptr<Framef> mpRoot;
};

#endif // __ROBOT_HPP_