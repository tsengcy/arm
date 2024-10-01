#ifndef __ENDEFFECTOR_HPP_
#define __ENDEFFECTOR_HPP_

#include <iostream>
#include <memory>
#include "robotics/frame.hpp"

class endeffector
{
public:
    endeffector(std::shared_ptr<Framef>);

private:
    std::shared_ptr<Framef> mpFrame;
};


#endif // __ENDEFFECTOR_HPP_