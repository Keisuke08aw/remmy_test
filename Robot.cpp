#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/LU"
#include <math.h>

void Robot::setJoint1(float x, float y, float z, float rad)
{
    joint1_vec << x, y, z, rad;
}
Eigen::MatrixXd Robot::getJoint1()
{
    return joint1_vec;
}
void Robot::setJoint2(float x, float y, float z, float rad)
{
    joint2_vec << x, y, z, rad;
}
Eigen::MatrixXd Robot::getJoint2()
{
    return joint2_vec;
}
void Robot::setJoint3(float x, float y, float z, float rad)
{
    joint3_vec << x, y, z, rad;
}
Eigen::MatrixXd Robot::getJoint3()
{
    return joint1_vec;
}

Eigen::MatrixXd Robot::joint1_vec(3, 1);
Eigen::MatrixXd Robot::joint2_vec(3, 1);
Eigen::MatrixXd Robot::joint3_vec(3, 1);

//ある関節(θ1, θ2, θ3, θ4)を入れて、今の手先座標(x,y,z)を返す関数
std::vector<double> Robot::direct_kinematics(Eigen::MatrixXd &joint_theta_vec)
{
    double rad1 = joint_theta_vec[0];
    double rad2 = joint_theta_vec[1];
    double rad3 = joint_theta_vec[2];

    Eigen::MatrixXd T_0_4(4, 4);

    Eigen::MatrixXd T_0_1(4, 4);
    Eigen::MatrixXd T_1_2(4, 4);
    Eigen::MatrixXd T_2_3(4, 4);
    Eigen::MatrixXd T_3_4(4, 4);

    T_0_1 << cos(rad1), -sin(rad1), 0, 0,
        sin(rad1), cos(rad1), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    T_1_2 << cos(rad2), sin(rad2), 0, 10,
        0, 0, -1, 0,
        -sin(rad2), cos(rad2), 0, 0,
        0, 0, 0, 1;

    T_2_3 << cos(rad3), -sin(rad3), 0, 5,
        sin(rad3), cos(rad3), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    T_3_4 << 1, 0, 0, 5,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4;

    std::vector<double> p_vec;
    double px = T_0_4(0, 3);
    double py = T_0_4(1, 3);
    double pz = T_0_4(2, 3);
    p_vec.push_back(px);
    p_vec.push_back(py);
    p_vec.push_back(pz);

    return p_vec;
}

//ある手先座標(x,y,z)を入れて、次の関節の角度(θ1, θ2, θ3, θ4)を返す関数
Eigen::MatrixXd inverse_kinematics(std::vector<double> &target_vec)
{
    Robot robot;

    Eigen::MatrixXd q_i_1(3, 1);
    Eigen::MatrixXd qi(3, 1);
    Eigen::MatrixXd learning_rate(3, 1);
    Eigen::MatrixXd inv_Jacobian(3, 3);
    Eigen::MatrixXd Jaconvian(3, 3);
    Eigen::MatrixXd ri(3, 3);

    learning_rate << 0.01, 0.01, 0.01;

    float rad1 = robot.getJoint1()(3);
    float rad2 = robot.getJoint2()(3);
    float rad3 = robot.getJoint3()(3);

    qi << rad1, rad2, rad3;

    float delta_r1 = sqrt(pow(target_vec[0] - robot.getJoint1()(0), 2) + pow(target_vec[1] - robot.getJoint1()(1), 2) + pow(target_vec[2] - robot.getJoint1()(2), 2));
    float delta_r2 = sqrt(pow(target_vec[0] - robot.getJoint2()(0), 2) + pow(target_vec[1] - robot.getJoint2()(1), 2) + pow(target_vec[2] - robot.getJoint2()(2), 2));
    float delta_r3 = sqrt(pow(target_vec[0] - robot.getJoint3()(0), 2) + pow(target_vec[1] - robot.getJoint3()(1), 2) + pow(target_vec[2] - robot.getJoint3()(2), 2));

    ri << delta_r1, delta_r2, delta_r3;

    Jaconvian(0, 0) = -sin(rad1) * cos(rad2) * (5 * cos(rad3) + 5) + (-sin(rad1)) * (5 * sin(rad2) * sin(rad3) + 10);
    Jaconvian(0, 1) = cos(rad1) * (-sin(rad2)) * (5 * cos(rad3) + 5) + 0;
    Jaconvian(0, 2) = cos(rad1) * cos(rad2) * 5 * (-sin(rad3)) + cos(rad1) * 5 * sin(rad2) * cos(rad3);

    Jaconvian(1, 0) = 5 * cos(rad1) * cos(rad2) * (cos(rad3) + 1) + 5 * cos(rad1) * (sin(rad2) * sin(rad3) + 2);
    Jaconvian(1, 1) = 5 * sin(rad1) * (-sin(rad2)) * (cos(rad3) + 1) + 5 * sin(rad1) * cos(rad2) * sin(rad3);
    Jaconvian(1, 2) = 5 * sin(rad1) * cos(rad2) * (-sin(rad3)) + 5 * sin(rad1) * sin(rad2) * cos(rad3);

    Jacobian(2, 0) = 0.0;
    Jacobian(2, 1) = -5 * cos(rad2) * (cos(rad3) + 1) - 5 * cos(rad2) * sin(rad3);
    Jacobian(2, 2) = -5sin(rad2) * (-sin(rad3)) + 5 * cos(rad2) * cos(rad3);

    inv_Jacobian = Jacobian.inverse();

    q_i_1 = qi + learning_rate * inv_Jacobian * ri;

    return q_i_1;
}
