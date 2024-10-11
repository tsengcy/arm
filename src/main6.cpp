#include <robotics/dynrobotarm.hpp>

int main()
{
    float L1 = 1;
    float L2 = 2;
    std::vector<float>     a{0, 1};
    std::vector<float> alpha{M_PI/2, 0};
    std::vector<float>     d{0, 0};
    std::vector<float> theta{0, 0};
    std::vector<float> upper{100, 100};
    std::vector<float> lower{-100, -100};
    std::vector<int> parentId{-1, 0};
    std::vector<FRAMETYPE> frametype{FRAMETYPE::ACTIVE, FRAMETYPE::ACTIVE};
    std::vector<int> eeparent{1};
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0,3) = L2;
    std::vector<Eigen::Matrix4f> vtrans{trans};

    std::vector<float> vmass{1, 1};
    Eigen::Matrix4f Tcom = Eigen::Matrix4f::Identity();
    Tcom(0,3) = L1;
    Eigen::Matrix4f Tcom2 = Eigen::Matrix4f::Identity();
    Tcom2(0,3) = L2;
    std::vector<Eigen::Matrix4f> vTCOM{Tcom, Tcom2};
    Eigen::Matrix3f inertia = Eigen::Matrix3f::Zero();
    // inertia(2,2) = 1;
    std::vector<Eigen::Matrix3f> vInertia{inertia, inertia};

    dynRobotArm* rbt = new dynRobotArm(a, alpha, d, theta, upper, lower, parentId, 
                                       DH::MODIFIED, frametype, vmass, vInertia, vTCOM, vtrans, eeparent);
    std::cout << "finish initialize\n";

    // std::cout << Frame::
    Eigen::VectorXf q(2), qd(2), qdd(2);
    q << 0, 0;
    qd << 0, 0;
    qdd << 0, 10;
    // angle << 0, 0;
    // rbt->set_q(angle);

    // std::cout << "posse\n" << rbt->get_EEPose() << std::endl;
    rbt->setGravity(true);
    rbt->Inverse_Dynamic(q, qd, qdd);
    // rbt->property();
    Eigen::VectorXf torque = rbt->get_Torque();
    std::cout << "Torque:\n" << torque << std::endl;

    rbt->Forward_Dynamic(q, qd, torque);

    std::cout << "acceleration\n" << rbt->get_qdd() << std::endl;

    delete(rbt);

    
}