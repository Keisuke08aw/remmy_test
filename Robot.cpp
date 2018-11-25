#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/LU"
#include <math.h>
#include "Robot.h"
#include <iostream>
#define _USE_MATH_DEFINES

float Robot::get_Joint1Angle()
{
    return joint1_angle;
}

float Robot::get_Joint2Angle()
{
    return joint2_angle;
}

float Robot::get_Joint3Angle()
{
    return joint3_angle;
}

Eigen::MatrixXd Robot::get_Joint4_vec()
{
    return joint4_vec;
}

void Robot::set_Joint1Angle(float radian)
{
    joint1_angle = radian;
}

void Robot::set_Joint2Angle(float radian)
{
    joint2_angle = radian;
}

void Robot::set_Joint3Angle(float radian)
{
    joint3_angle = radian;
}
void Robot::set_Joint4_vec(float x, float y, float z)
{
    joint4_vec << x, y,z;
}

void Robot::set_hoge(int num)
{
    hoge = num;
}

int Robot::get_hoge()
{
    return hoge;
}

int Robot::hoge;
float Robot::joint1_angle = M_PI / 2;
float Robot::joint2_angle = -M_PI / 2;
float Robot::joint3_angle = M_PI / 4;
Eigen::MatrixXd Robot::joint4_vec(3, 1);

//ある関節(θ1, θ2, θ3)を入れて、今の手先座標(x,y,z)を返す関数
std::vector<float> Robot::direct_kinematics(Eigen::MatrixXd vec_joint_angle)
{
    Robot robot;

    float rad1 = vec_joint_angle(0);
    float rad2 = vec_joint_angle(1);
    float rad3 = vec_joint_angle(2);

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

    std::vector<float> vec_p;
    float px = T_0_4(0, 3);
    float py = T_0_4(1, 3);
    float pz = T_0_4(2, 3);

    vec_p.push_back(px);
    vec_p.push_back(py);
    vec_p.push_back(pz);

    return vec_p;
}

//ある手先座標(x,y,z)を入れて、次の関節の角度(θ1, θ2, θ3, θ4)を返す関数
Eigen::MatrixXd Robot::inverse_kinematics(std::vector<float> target_vec)
{
    // printf("AAAAAAAAAAAAAAAAAAAA\r\n");
    Robot robot;

    Eigen::MatrixXd qi_1(3, 1);
    Eigen::MatrixXd qi(3, 1);
    Eigen::MatrixXd learning_rate(3, 3);
    Eigen::MatrixXd inv_Jacobian(3, 3);
    Eigen::MatrixXd Jacobian(3, 3);
    Eigen::MatrixXd ri(3, 1);

    float target_x = target_vec[0];
    float target_y = target_vec[1];
    float target_z = target_vec[2];

    Eigen::MatrixXd value_matrix = robot.get_Joint4_vec();
    float eng_effecotr_x = value_matrix(0);
    float eng_effecotr_y = value_matrix(1);
    float eng_effecotr_z = value_matrix(2);

    float delta_x = sqrt(pow((target_x - eng_effecotr_x), 2));
    // printf("target%f\r\n", target_x);

    // printf("END%f\r\n", eng_effecotr_x);
    // printf("DELTA%f\r\n", delta_x);

    float delta_y = sqrt(pow((target_y - eng_effecotr_y), 2));
    float delta_z = sqrt(pow((target_z - eng_effecotr_z), 2));

    float rad1 = robot.get_Joint1Angle();
    float rad2 = robot.get_Joint2Angle();
    float rad3 = robot.get_Joint3Angle();

    // printf("rad1 %f\r\n", rad1);
    // printf("rad2 %f\r\n", rad2);
    // printf("rad3 %f\r\n", rad3);

    // printf("BBBBBBBBBBBBBBBBBB\r\n");

    Jacobian(0, 0) = -sin(rad1) * cos(rad2) * (5 * cos(rad3) + 5) + (-sin(rad1)) * (5 * sin(rad2) * sin(rad3) + 10);
    Jacobian(0, 1) = cos(rad1) * (-sin(rad2)) * (5 * cos(rad3) + 5) + 0;
    Jacobian(0, 2) = cos(rad1) * cos(rad2) * 5 * (-sin(rad3)) + cos(rad1) * 5 * sin(rad2) * cos(rad3);

    Jacobian(1, 0) = 5 * cos(rad1) * cos(rad2) * (cos(rad3) + 1) + 5 * cos(rad1) * (sin(rad2) * sin(rad3) + 2);
    Jacobian(1, 1) = 5 * sin(rad1) * (-sin(rad2)) * (cos(rad3) + 1) + 5 * sin(rad1) * cos(rad2) * sin(rad3);
    Jacobian(1, 2) = 5 * sin(rad1) * cos(rad2) * (-sin(rad3)) + 5 * sin(rad1) * sin(rad2) * cos(rad3);

    Jacobian(2, 0) = 0.0;
    Jacobian(2, 1) = -5 * cos(rad2) * (cos(rad3) + 1) - 5 * cos(rad2) * sin(rad3);
    Jacobian(2, 2) = -5 * sin(rad2) * (-sin(rad3)) + 5 * cos(rad2) * cos(rad3);

    // printf("Jacobian\r\n");

    // std::cout << Jacobian << std::endl;
    // Eigen::MatrixXd transposed_Jacobian = Jacobian.transpose();
    // std::cout << transposed_Jacobian << std::endl;
    // Eigen::MatrixXd JJ = (Jacobian * transposed_Jacobian);
    // Eigen::MatrixXd inv_JJ = JJ.inverse();
    // Eigen::MatrixXd pseudo_Jacobian = transposed_Jacobian * inv_JJ;
    // std::cout << Jacobian << std::endl;
    // std::cout << JJ << std::endl;
    // std::cout << inv_JJ << std::endl;
    // std::cout << pseudo_Jacobian << std::endl;

    // printf("CCCCCCCCCCCCCCCCCCCCC\r\n\r\n");

    //逆ヤコビアン
    // printf("inv_Jacobian\r\n");
    inv_Jacobian = Jacobian.inverse();
    // std::cout << inv_Jacobian << std::endl;

    // printf("DDDDDDDDDDDDDDDDDD\r\n");

    //今の各関節のjointのangleをget
    qi << rad1, rad2, rad3;

    // printf("EEEEEEEEEEEEEEEEEEEEE\r\n");

    //手先が目標に対してどれだけ離れているか計算
    ri << delta_x, delta_y, delta_z;

    // printf("FFFFFFFFFFFFFFFFFFF\r\n");

    //学習率
    learning_rate << 0.1, 0, 0,
                    0, 0.1, 0,
                0, 0, 0.1;

    // printf("GGGGGGGGGGGGGGGGGGGGG\r\n");

    // std::cout << qi << std::endl;
    // std::cout << learning_rate << std::endl;
    // std::cout << inv_Jacobian << std::endl;
    // std::cout << ri << std::endl;

    // printf("HHHHHHHHHHHHHHHHHH\r\n");

    qi_1 = qi + learning_rate * inv_Jacobian * ri;
    // Eigen::MatrixXd v1 = learning_rate * inv_Jacobian;
    // Eigen::MatrixXd v2 = v1 * ri;
    // printf("HHHHHHHHHHHHHHHHHH\r\n");

    // qi_1 = qi + v2;

    // printf("IIIIIIIIIIIIIIIIIIII\r\n");

    // std::cout << qi_1 << std::endl;

    // printf("JJJJJJJJJJJJJJJJJJJ\r\n");

    return qi_1;
}
