#include <robotics/utils.hpp>

std::vector<float> mathfunction::EigenVectorToStdVector(Eigen::VectorXf vec)
{
    std::vector<float> ans(vec.rows(), 0);
    for(int i=0; i<vec.rows(); i++)
    {
        ans[i] = vec(i);
    }
    
    return ans;
}

Eigen::VectorXf mathfunction::PoseToPosOri(Eigen::Matrix4f Pose)
{
    Eigen::VectorXf PosOri(6);
    Eigen::Matrix3f mat = Pose.block(0, 0, 3, 3);
    PosOri.block(0, 0, 3, 1) = SO3Toso3(mat);
    PosOri.block(3, 0, 3, 1) = Pose.block(0, 3, 3, 1);

    return PosOri;
}

Eigen::Vector3f mathfunction::PoseToPos(Eigen::Matrix4f Pose)
{
    return Pose.block(0, 3, 3, 1);
}

Eigen::Matrix4f mathfunction::PosOriToPose(Eigen::VectorXf PosOri)
{
    Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
    Pose.block(0, 0, 3, 3) = so3ToSO3(PosOri.block(0, 0, 3, 1));
    Pose.block(0, 3, 3, 1) = PosOri.block(3, 0, 3, 1);

    return Pose;
}

Eigen::Matrix3f mathfunction::so3ToSO3(Eigen::Vector3f so)
{
    float angle = so.norm();
    Eigen::Vector3f axis = so / angle;

    Eigen::Matrix3f ans; 
    ans << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    ans = ans + sin(angle) * skew(axis) + (1 - cos(angle)) * skew(axis) * skew(axis);

    return ans;
}

Eigen::Vector3f mathfunction::SO3Toso3(Eigen::Matrix3f SO)
{
    float angle = acos((SO.trace() - 1) / 2.0);
    if(angle > M_PI || angle < -M_PI) throw std::invalid_argument("error of rotation angle");
    if(angle < 0.0001)
    {
        Eigen::Vector3f axis;
        axis << 0,0,0;
        return axis;   
    }
    else
    {
        Eigen::Matrix3f mat = (SO - SO.transpose())/(2 * sin(angle));
        Eigen::Vector3f axis = mathfunction::nskew(mat);
        return axis * angle;
    }
}

Eigen::Matrix4f mathfunction::se3ToSE3(Eigen::VectorXf se)
{
    Eigen::Matrix4f ans = Eigen::Matrix4f::Identity();
    Eigen::Vector3f axis = se.block(0, 0, 3, 1);
    ans.block(0, 0, 3, 3) = so3ToSO3(axis);

    float angle = axis.norm();
    axis = axis / angle;

    Eigen::Matrix3f G = Eigen::Matrix3f::Identity() * angle 
                               + (1 - cos(angle)) * skew(axis) + (angle - sin(angle)) * skew(axis) * skew(axis);
    ans.block(0, 3, 3, 1) = G * se.block(3, 0, 3, 1);

    return ans;
}

Eigen::VectorXf mathfunction::SE3Tose3(Eigen::Matrix4f SE)
{
    Eigen::VectorXf ans = Eigen::VectorXf::Zero(6);
    Eigen::Matrix3f mat = SE.block(0,0,3,3);
    ans.block(0, 0, 3, 1) = SO3Toso3(mat);

    float angle = ans.block(0, 0, 3, 1).norm();
    Eigen::Vector3f axis = ans.block(0, 0, 3, 1) / angle;

    Eigen::Matrix3f G = Eigen::Matrix3f::Identity() * angle 
                               + (1 - cos(angle)) * skew(axis) + (angle - sin(angle)) * skew(axis) * skew(axis);

    ans.block(3, 0, 3, 1) = G.inverse() * SE.block(0, 3, 3, 1);
    return ans;
}

void mathfunction::se3ToScrew(Eigen::VectorXf se, Eigen::VectorXf& Screw, float& theta)
{
    theta = se.block(0, 0, 3, 1).norm();
    if(theta < 0.0001)
    {
        theta = se.block(3, 0, 3, 1).norm();
        Screw = se / theta;
    }
    else
    {
        Screw = se / theta;
    }
}

void mathfunction::SE3ToScrew(Eigen::Matrix4f SE, Eigen::VectorXf& Screw, float& theta)
{
    Eigen::VectorXf se = SE3Tose3(SE);
    std::cout << "se\n" << se << std::endl;
    se3ToScrew(se, Screw, theta);
}

Eigen::Matrix3f mathfunction::skew(Eigen::Vector3f vec)
{
    Eigen::Matrix3f ans;
    ans <<       0, -vec(2),  vec(1),
            vec(2),       0, -vec(0),
           -vec(1),  vec(0),       0;
    return ans;
}

Eigen::Vector3f mathfunction::nskew(Eigen::Matrix3f mat)
{
    Eigen::Vector3f vec;
    vec << mat(2, 1), mat(0, 2), mat(1, 0);

    return vec; 
}

Eigen::MatrixXf mathfunction::adjoint(Eigen::VectorXf PosOri)
{
    Eigen::Matrix4f Pose = PosOriToPose(PosOri);

    return adjoint(Pose);
}

Eigen::MatrixXf mathfunction::adjoint(Eigen::Matrix4f Pose)
{
    Eigen::MatrixXf adj = Eigen::MatrixXf::Zero(6, 6);
    adj.block(0, 0, 3, 3) = Pose.block(0, 0, 3, 3);
    adj.block(3, 3, 3, 3) = Pose.block(0, 0, 3, 3);
    Eigen::Vector3f Pos = Pose.block(0, 3, 3, 1);
    adj.block(3, 0, 3, 3) = skew(Pos) *  Pose.block(0, 0, 3, 3);

    return adj;
}

Eigen::MatrixXf mathfunction::LieBracket(Eigen::VectorXf v1)
{
    Eigen::MatrixXf ad = Eigen::MatrixXf::Zero(6,6);
    Eigen::Vector3f r = v1.block(0, 0, 3, 1);
    ad.block(0, 0, 3, 3) = skew(r);
    ad.block(3, 3, 3, 3) = skew(r);
    Eigen::Vector3f v = v1.block(3, 0, 3, 1);
    ad.block(3, 0, 3, 3) = skew(v);

    return ad;
}