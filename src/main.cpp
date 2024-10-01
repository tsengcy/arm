#include <iostream>
#include <robotics/frame.hpp>
#include <memory>

using FramePtr = std::shared_ptr<Framef>;

int main()
{
    FramePtr root(new Framef(1, 0, 0, 0));
    FramePtr f1(new Framef(1, 0, 0, 0, root));
    root->set_Child(f1);


    root->property();
    f1->property();

     
}