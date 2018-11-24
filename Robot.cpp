#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include <math.h>

//ある関節(θ1, θ2, θ3, θ4)を入れて、手先座標(x,y,z)を返す関数
std::vector<double> Robot::direct_kinematics(std::vector<double> &joint_theta_vec)
{
    rad1 = joint_theta_vec[0];
    rad2 = joint_theta_vec[1];
    rad3 = joint_theta_vec[2];

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

    std::vector<double> end_effector_vec;
    end_effector_vec.push_back(T_0_4(0, 3));
    end_effector_vec.push_back(T_0_4(1, 3));
    end_effector_vec.push_back(T_0_4(2, 3));

    return end_effector_vec;
}

//ある手先座標(x,y,z)を入れて、すべての関節(θ1, θ2, θ3, θ4)を返す関数
std::vector<double> inverse_kinematics(std::vector<double> &target_vec)
{
    Robot robot;
    

    float x, y, z;
    float rad1, rad2, rad3;
    float theta1, theta2, theta3;

    Eigen::MatrixXd Jaconvian(3, 3);
    Jaconvian(0, 0) = -sin(rad1) * cos(rad2) * (5 * cos(rad3) + 5) + (-sin(rad1)) * (5 * sin(rad2) * sin(rad3) + 10);
    Jaconvian(0, 1) = cos(rad1) * (-sin(rad2)) * (5 * cos(rad3) + 5) + 0;
    Jaconvian(0, 2) = cos(rad1) * cos(rad2) * 5 * (-sin(rad3)) + cos(rad1) * 5 * sin(rad2) * cos(rad3);

    Jaconvian(1, 0) = 5 * cos(rad1) * cos(rad2) * (cos(rad3) + 1) + 5 * cos(rad1) * (sin(rad2) * sin(rad3) + 2);
    Jaconvian(1, 1) = 5 * sin(rad1) * (-sin(rad2)) * (cos(rad3) + 1) + 5 * sin(rad1) * cos(rad2) * sin(rad3);
    Jaconvian(1, 2) = 5 * sin(rad1) * cos(rad2) * (-sin(rad3)) + 5 * sin(rad1) * sin(rad2) * cos(rad3);

    Eigen::MatrixXd learning_rate(3, 1);
    learning_rate << 0.01,
        0.01,
        0.01;

    Eigen::MatrixXd q_i_1(4, 1);

    q_i_1 = qi + k * inv_Jacobian;

    ;
}

std::vector<double> inv_Jacobian(std::vector<double> &target_vec)
{
}
