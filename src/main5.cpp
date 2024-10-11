#include <iostream>
#include <robotics/robotarm.hpp>
#include <memory>
#include <robotics/utils.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>


int main()
{
    std::vector<float>     a{0, 1, -1};
    std::vector<float> alpha{M_PI/2, -M_PI/2, -M_PI/2};
    std::vector<float>     d{0, 0, 0};
    std::vector<float> theta{0, M_PI, M_PI};
    std::vector<float> upper{100.0 / 180 * M_PI, 100.0 / 180 * M_PI, 100.0 / 180 * M_PI};
    std::vector<float> lower{-100.0 / 180 * M_PI, -100.0 / 180 * M_PI, -100.0 / 180 * M_PI};
    std::vector<int> parentId{-1, 0, 1};
    std::vector<FRAMETYPE> frametype{FRAMETYPE::ACTIVE, FRAMETYPE::ACTIVE, FRAMETYPE::ACTIVE};
    std::vector<int> eeparent{2};
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = 1;
    std::vector<Eigen::Matrix4f> vtrans{trans};

    RobotArm* rbt = new RobotArm(a, alpha, d, theta, upper, lower, parentId, DH::MODIFIED,
                                   frametype, vtrans, eeparent);

    rbt->property();
    std::cout << "Pose\n" << rbt->get_EEPose() << std::endl;

    // std::vector<float> angle{M_PI/2, 0};
    // rbt->set_q(angle);
    // std::cout << "Pose\n" << rbt->get_EEPosOri() << std::endl;

    // Eigen::VectorXf eePos(3);
    // eePos << sqrt(2.0), 0, 1.0 + sqrt(2.0);

    // Eigen::VectorXf IKAngle = rbt->IK_Pos(eePos);
    // std::cout << "ee pos \n" << eePos << "\nangle\n";
    // std::cout << IKAngle << std::endl;

    Eigen::VectorXf eePos(6);
    eePos << 1.2091996, -1.2091996, 1.2091996, sqrt(2.0), 0, 1.0 + sqrt(2.0);

    Eigen::VectorXf IKAngle = rbt->IK_PosOri(eePos);
    std::cout << "ee pos \n" << eePos << "\nangle\n";
    std::cout << IKAngle * 180 / M_PI << std::endl;
    


    delete(rbt);
}