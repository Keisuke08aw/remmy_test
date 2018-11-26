#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/LU"
#include <math.h>
#include "Robot.h"
#include <iostream>
#define _USE_MATH_DEFINES

std::vector<float> Robot::get_target_vec()
{
    return target_vec;
}

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

Eigen::MatrixXd Robot::get_EndEffector_vec()
{
    return EndEffector_vec;
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
void Robot::set_EndEffector_vec(float x, float y, float z)
{
    EndEffector_vec << x, y, z;
}

// void Robot::set_Target_vec(std::vector<unsigned char> &data)
// {
//     printf("/////Robot.cpp set_Target_vec/////\r\n");

//     // convert bytes back to float
//     uint8_t bytes1[sizeof(float)];
//     uint8_t bytes2[sizeof(float)];
//     uint8_t bytes3[sizeof(float)];

//     for (int i = 0; i < 4; i++)
//     {
//         bytes1[i] = data[i];
//         bytes2[i] = data[i + 4];
//         bytes3[i] = data[i + 8];
//     }
//     float x_p = *(float *)(bytes1); // convert bytes back to float
//     float y_p = *(float *)(bytes2); // convert bytes back to float
//     float z_p = *(float *)(bytes3); // convert bytes back to float

//     printf("float %.1f\r\n", x_p);
//     printf("float %.1f\r\n", y_p);
//     printf("float %.1f\r\n", z_p);

//     std::vector<float> t_vec;

//     t_vec.push_back(x_p);
//     t_vec.push_back(y_p);
//     t_vec.push_back(z_p);

//     printf("t_vec x is set %.2f\r\n", t_vec[0]);
//     printf("t_vec y is set %.2f\r\n", t_vec[1]);
//     printf("t_vec z is set %.2f\r\n", t_vec[2]);

//     target_vec = t_vec;
//     printf("/////Robot.cpp set_Target_vec END /////\r\n\r\n");
// }

float Robot::joint1_angle = M_PI / 2;
float Robot::joint2_angle = M_PI / 2;
float Robot::joint3_angle = M_PI / 2;
std::vector<float> Robot::target_vec;
Eigen::MatrixXd Robot::EndEffector_vec(3, 1);

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
int Robot::inverse_kinematics(std::vector<unsigned char> &data)
{
    printf("/////Robot.cpp inverse_Kinematics/////\r\n");
    // convert bytes back to float
    uint8_t bytes1[sizeof(float)];
    uint8_t bytes2[sizeof(float)];
    uint8_t bytes3[sizeof(float)];

    for (int i = 0; i < 4; i++)
    {
        bytes1[i] = data[i];
        bytes2[i] = data[i + 4];
        bytes3[i] = data[i + 8];
    }
    //これがターゲットのx,y,z座標
    float x_p = *(float *)(bytes1); // convert bytes back to float
    float y_p = *(float *)(bytes2); // convert bytes back to float
    float z_p = *(float *)(bytes3); // convert bytes back to float

    printf("float %.1f\r\n", x_p);
    printf("float %.1f\r\n", y_p);
    printf("float %.1f\r\n", z_p);

    int result;
    Robot robot;
    Eigen::MatrixXd qi_1(3, 1);
    Eigen::MatrixXd qi(3, 1);
    Eigen::MatrixXd learning_rate(3, 3);
    Eigen::MatrixXd inv_Jacobian(3, 3);
    Eigen::MatrixXd Jacobian(3, 3);
    Eigen::MatrixXd ri(3, 1);
    float target_x = x_p;
    float target_y = y_p;
    float target_z = z_p;
    Eigen::MatrixXd end_effector_vec;
    float end_x;
    float end_y;
    float end_z;
    float delta_xyz;
    float delta_x;
    float delta_y;
    float delta_z;
    float rad1;
    float rad2;
    float rad3;

    //学習率
    learning_rate << 1.8, 0, 0,
        0, 1.8, 0,
        0, 0, 1.8;
    try
    {
        while (true)
        {
            rad1 = robot.get_Joint1Angle();
            rad2 = robot.get_Joint2Angle();
            rad3 = robot.get_Joint3Angle();

            //目標座標と今の手の位置との距離を求める
            delta_xyz = sqrt(pow((target_x - end_effector_vec(0)), 2) + pow((target_y - end_effector_vec(1)), 2) + pow((target_z - end_effector_vec(2)), 2));
            if (delta_xyz < 0.5)
            {
                qi << rad1, rad2, rad3;
                std::vector<float> nearest_end_vec = robot.direct_kinematics(qi);
                robot.set_EndEffector_vec(nearest_end_vec[0], nearest_end_vec[1], nearest_end_vec[2]);
                result = true;
                break;
            }

            Jacobian(0, 0) = -sin(rad1) * cos(rad2) * (5 * cos(rad3) + 5) + (-sin(rad1)) * (5 * sin(rad2) * sin(rad3) + 10);
            Jacobian(0, 1) = cos(rad1) * (-sin(rad2)) * (5 * cos(rad3) + 5) + 0;
            Jacobian(0, 2) = cos(rad1) * cos(rad2) * 5 * (-sin(rad3)) + cos(rad1) * 5 * sin(rad2) * cos(rad3);

            Jacobian(1, 0) = 5 * cos(rad1) * cos(rad2) * (cos(rad3) + 1) + 5 * cos(rad1) * (sin(rad2) * sin(rad3) + 2);
            Jacobian(1, 1) = 5 * sin(rad1) * (-sin(rad2)) * (cos(rad3) + 1) + 5 * sin(rad1) * cos(rad2) * sin(rad3);
            Jacobian(1, 2) = 5 * sin(rad1) * cos(rad2) * (-sin(rad3)) + 5 * sin(rad1) * sin(rad2) * cos(rad3);

            Jacobian(2, 0) = 0.0;
            Jacobian(2, 1) = -5 * cos(rad2) * (cos(rad3) + 1) - 5 * cos(rad2) * sin(rad3);
            Jacobian(2, 2) = -5 * sin(rad2) * (-sin(rad3)) + 5 * cos(rad2) * cos(rad3);

            end_effector_vec = robot.get_EndEffector_vec();

            delta_x = sqrt(pow((target_x - end_effector_vec(0)), 2));
            delta_y = sqrt(pow((target_y - end_effector_vec(1)), 2));
            delta_z = sqrt(pow((target_z - end_effector_vec(2)), 2));

            qi << rad1, rad2, rad3;
            ri << delta_x, delta_y, delta_z;
            inv_Jacobian = Jacobian.inverse();

            //新しいθ1~θ3
            qi_1 = qi + learning_rate * inv_Jacobian * ri;
            //θ1~θ3を登録
            robot.set_Joint1Angle(qi_1(0));
            robot.set_Joint1Angle(qi_1(1));
            robot.set_Joint1Angle(qi_1(2));

            std::vector<float> now_end_effector_vec = robot.direct_kinematics(qi_1);
            printf("%.2f", now_end_effector_vec[0]);
            printf("%.2f", now_end_effector_vec[1]);
            printf("%.2f", now_end_effector_vec[2]);

            robot.set_EndEffector_vec(now_end_effector_vec[0], now_end_effector_vec[1], now_end_effector_vec[2]);
        }
    }

    catch (std::exception e)
    {
        printf("ERROR HAPPENS\r\n");
        result = false;
    }
    printf("/////Robot.cpp inverse_Kinematics/////\r\n");
    return result;
}
