#include <robotics/dynrobotarm.hpp>

int main()
{
    std::vector<float>     a{0};
    std::vector<float> alpha{M_PI/2};
    std::vector<float>     d{0};
    std::vector<float> theta{0};
    std::vector<float> upper{100};
    std::vector<float> lower{-100};
    std::vector<int> parentId{-1};
    std::vector<FRAMETYPE> frametype{FRAMETYPE::ACTIVE};
    std::vector<int> eeparent{0};
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0,3) = 1;
    std::vector<Eigen::Matrix4f> vtrans{trans};

    std::vector<float> vmass{1};
    Eigen::Matrix4f Tcom = Eigen::Matrix4f::Identity();
    Tcom(0,3) = 1;
    std::vector<Eigen::Matrix4f> vTCOM{Tcom};
    Eigen::Matrix3f inertia = Eigen::Matrix3f::Zero();
    // inertia(2,2) = 1;
    std::vector<Eigen::Matrix3f> vInertia{inertia};

    dynRobotArm* rbt = new dynRobotArm(a, alpha, d, theta, upper, lower, parentId, 
                                       DH::MODIFIED, frametype, vmass, vInertia, vTCOM, vtrans, eeparent);
    std::cout << "finish initialize\n";

    // std::cout << Frame::
    Eigen::VectorXf angle(1);
    angle << 0;
    // angle << 0, 0;
    // rbt->set_q(angle);

    // std::cout << "posse\n" << rbt->get_EEPose() << std::endl;;

    rbt->Inverse_Dynamic(angle, Eigen::VectorXf::Zero(1), Eigen::VectorXf::Zero(1));
    // rbt->property();
    std::cout << "Torque:\n" << rbt->get_Torque() << std::endl;

    std::cout << "Pose: \n" << rbt->get_EEPose() << std::endl;



    delete(rbt);

    
}