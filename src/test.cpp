#include <iostream>
#include <robotics/utils.hpp>

int main()
{
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0,3) = 1;

    std::cout << "mat\n" << mat << std::endl;
    std::cout << mathfunction::adjoint(mat) << std::endl; 
}