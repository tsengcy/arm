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
                                       DH::MODIFIED, frametype, vmass, vInertia, vTCOM, vtrans, eeparent, true);
    std::cout << "finish initialize\n";

    // std::cout << Frame::
    Eigen::VectorXf q(2), qd(2), qdd(2);
    q << 0, 0;
    qd << 0, 0;
    qdd << 0, 0;
    // angle << 0, 0;
    // rbt->set_q(angle);

    // std::cout << "posse\n" << rbt->get_EEPose() << std::endl;

    Eigen::VectorXf force(6);
    force << 0, 0, 1, 0, -0.5, 0;
    int id = 1;
    Eigen::Vector3f forcePos;
    forcePos << L2, 0, 0;

    std::vector<Eigen::VectorXf> externalForce{force};
    std::vector<int> externalForceId{id};
    std::vector<Eigen::Vector3f> externalForcePos{forcePos};

    

    rbt->Inverse_Dynamic(q, qd, qdd, externalForce, externalForcePos, externalForceId);
    // rbt->property();
    std::cout << "Torque:\n" << rbt->get_Torque() << std::endl;

    // std::cout << "Pose: \n" << rbt->get_EEPose() << std::endl;

    Eigen::Vector2f torque = Eigen::Vector2f::Zero();

    torque(0) = (vmass[0] * L1 * L1 + vmass[1]*(L1 * L1 + 2 * L1 * L2 * cos(q(1)) + L2 * 2)) * qdd(0) +
                vmass[1] * (L1 * L2 * cos(q[1]) + L2 * L2) * qdd(1) - vmass[1] * L1 * L2 * sin(q(1)) * (2 * qd(0) * qd(1) + qd(1) * qd(1)) +
                (vmass[1] + vmass[0]) * L1 * 10 * cos(q[0]) + vmass[1] * 10 * L2 * cos(q(0) + q(1));
    
    torque(1) = vmass[1] * (L1 * L2 * cos(q(1)) + L2 * L2) * qdd(0) + vmass[1] * L2 * L2 * qdd(1) + vmass[1] * L1 * L2 * qd(0) * qd(0) * sin(q(1)) +
                vmass[1] * 10 * L2 * cos(q(1) + q(0));

    std::cout << "torque:\n" << torque << std::endl;

    // std::cout << "pose\n" << rbt->get_EEPose() << std::endl;
    // std::cout << "twist\n" << rbt->get_GlobalTwist() << std::endl;
    // std::cout << "twistd\n" << rbt->get_GlobalTwistd() << std::endl;
    delete(rbt);

    
}