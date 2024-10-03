#include <iostream>
#include <robotics/robotarm.hpp>
#include <memory>
#include <robotics/utils.hpp>


int main()
{
    std::vector<float>     a{0, 1};
    std::vector<float> alpha{0, 0};
    std::vector<float>     d{0, 0};
    std::vector<float> theta{0, 0};
    std::vector<float> upper{100, 100};
    std::vector<float> lower{-100, -100};
    std::vector<int> parentId{-1, 0};
    std::vector<FRAMETYPE> frametype{FRAMETYPE::ACTIVE, FRAMETYPE::ACTIVE};
    std::vector<int> eeparent{1};
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = 1;
    std::vector<Eigen::Matrix4f> vtrans{trans};

    RobotArmf* rbt = new RobotArmf(a, alpha, d, theta, upper, lower, parentId, DH::MODIFIED,
                                   frametype, vtrans, eeparent);

    rbt->property();
    std::cout << "Pose\n" << rbt->get_EEPose() << std::endl;

    std::vector<float> angle{M_PI/4, M_PI/4};
    rbt->set_Angle(angle);
    std::cout << "Pose\n" << rbt->get_EEPosOri() << std::endl;


    delete(rbt);
}