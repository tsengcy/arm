#include <robotics/dynrobotarm.hpp>

int main()
{
    std::vector<float>     a{0, 0};
    std::vector<float> alpha{0, M_PI/2};
    std::vector<float>     d{0, 1};
    std::vector<float> theta{M_PI/2, M_PI};
    std::vector<float> upper{100, 100};
    std::vector<float> lower{-100, -100};
    std::vector<int> parentId{-1, 0};
    std::vector<FRAMETYPE> frametype{FRAMETYPE::ACTIVE, FRAMETYPE::ACTIVE};
    std::vector<int> eeparent{1};
    Eigen::Matrix4f trans = Eigen::Matrix4f::Zero();
    trans(3,3) = 1;
    trans(0,1) = -1;
    trans(1,2) = -1;
    trans(1,3) = 1;
    trans(2,0) = 1;
    std::cout << "trans\n" << trans << std::endl;
    std::vector<Eigen::Matrix4f> vtrans{trans};

    std::vector<float> vmass{2, 2};
    Eigen::Matrix4f Tcom1 = Eigen::Matrix4f::Zero();
    Tcom1(3,3) = 1;
    Tcom1(0,1) = 1;
    Tcom1(1,0) = -1;
    Tcom1(1,3) = -1;
    Tcom1(2,2) = 1;
    std::cout << "Tcom1\n" << Tcom1 << std::endl;

    std::vector<Eigen::Matrix4f> vTCOM{Tcom1, trans};
    Eigen::Matrix3f inertia1 = Eigen::Matrix3f::Zero();
    inertia1(1,1) = 4;
    inertia1(2,2) = 4;

    Eigen::Matrix3f inertia2 = Eigen::Matrix3f::Zero();
    inertia2(0,0) = 4;
    inertia2(1,1) = 4;
    std::vector<Eigen::Matrix3f> vInertia{inertia1, inertia2};

    dynRobotArm* rbt = new dynRobotArm(a, alpha, d, theta, upper, lower, parentId, 
                                       DH::MODIFIED, frametype, vmass, vInertia, vTCOM, vtrans, eeparent);
    std::cout << "finish initialize\n";

    // std::cout << Frame::
    Eigen::VectorXf q(2), qd(2), qdd(2);
    q << M_PI/4, M_PI/4;
    qd << 0, 0;
    qdd << 0, 0;
    // angle << 0, 0;
    // rbt->set_q(angle);

    // std::cout << "posse\n" << rbt->get_EEPose() << std::endl;;

    rbt->Inverse_Dynamic(q, qd, qdd);
    // rbt->property();
    std::cout << "Torque:\n" << rbt->get_Torque() << std::endl;

    std::cout << "Pose: \n" << rbt->get_EEPose() << std::endl;

    // Eigen::Vector2f torque = Eigen::Vector2f::Zero();

    // torque(0) = (vmass[0] * 1 + vmass[1]*(1 + 2 * cos(q(1)) + 1)) * qdd(0) +
    //             vmass[1] * (1 * cos(q[1]) + 1) * qdd(1) - vmass[1] * 1 * sin(q(1)) * (2 * qd(0) * qd(1) + qd(1) * qd(1)) +
    //             (vmass[1] + vmass[0]) * 1 * 9.81 * cos(q[0]) + vmass[1] * 9.81 * 1 * cos(q(0) + q(1));
    
    // torque(1) = vmass[1] * (1 * cos(q(1)) + 1) * qdd(0) + vmass[1] * 1 * qdd(1) + vmass[1] * 1 * qd(0) * qd(0) * sin(q(1)) +
    //             vmass[1] * 9.81 * 1 * cos(q(1) + q(0));

    // std::cout << "torque:\n" << torque << std::endl;

    delete(rbt);

    
}