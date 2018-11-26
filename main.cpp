#include <vector>
#include "Connection.h"
#include "Robot.h"
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES

std::vector<unsigned char> convert_Float_to_Byte(float num)
{
    std::vector<unsigned char> data;
    uint8_t bytes[sizeof(float)];
    *(float *)(bytes) = num; // convert float to bytes

    data.push_back(bytes[0]);
    data.push_back(bytes[1]);
    data.push_back(bytes[2]);
    data.push_back(bytes[3]);
    return data;
}

float convert_Byte_to_Float(std::vector<unsigned char> &data)
{
    uint8_t bytes[sizeof(float)];
    for (int i = 0; i < 4; i++)
    {
        bytes[i] = data[i];
    }
    float x_p = *(float *)(bytes); // convert bytes back to float
    return x_p;
}

int main(int argc, char const *argv[])
{
    Connection connection;
    //input.inをすべて読み込み、connectionのメンバ変数にセット

    connection.open();
    //すべての座標を取得
    std::vector<float> trajectory_vec_list = connection.get_trajectory();

    //trajectory_vec_list ={20.0, 0.0, 0.0, 17.0, 0.0, ....}
    float x = trajectory_vec_list[0];
    float y = trajectory_vec_list[1];
    float z = trajectory_vec_list[2];

    printf("\r\n");
    printf("First target x %.2f\r\n", x);
    printf("First target y %.2f\r\n", y);
    printf("First target z %.2f\r\n\r\n", z);

    std::vector<unsigned char> target_p;
    std::vector<unsigned char> target_x = convert_Float_to_Byte(x);
    std::vector<unsigned char> target_y = convert_Float_to_Byte(y);
    std::vector<unsigned char> target_z = convert_Float_to_Byte(z);

    target_p.insert(target_p.end(), target_x.begin(), target_x.end());
    target_p.insert(target_p.end(), target_y.begin(), target_y.end());
    target_p.insert(target_p.end(), target_z.begin(), target_z.end());

    printf("target bytes0 %d\r\n", target_p[0]);
    printf("target bytes1 %d\r\n", target_p[1]);
    printf("target bytes2 %d\r\n", target_p[2]);
    printf("target bytes3 %d\r\n\r\n", target_p[3]);

    //std::vector<unsigned char> &data
    //１点のみ送るだから、data.seiz() == 12;
    //target_p = {0, 0, 160, 65, 0, 0, 0, 0...}
    connection.send(target_p);

    Robot robot;

    Eigen::MatrixXd init_q(3, 1);

    float init_x = 0.0;
    float init_y = M_PI / 2;
    float init_z = -M_PI / 2;

    //関節の初期値を適当に決める
    init_q << init_x, init_y, init_z;

    //適当に決めたときの手先座標x,y,zを得る
    std::vector<float> init_end_effector_vec = robot.direct_kinematics(init_q);

    //適当に決めた関節に対する、手先座標x,y,zをsetする
    robot.set_Joint4_vec(init_end_effector_vec[0], init_end_effector_vec[1], init_end_effector_vec[2]);

    //手先座標とターゲット座標の差が0.5より小さくならなかったら
    while ((float)robot.delta(robot.get_target_vec()) >= 0.5)
    {
        float delta_xyz = robot.delta(robot.get_target_vec());
        printf("Delta %.2f\r\n", delta_xyz);

        Eigen::MatrixXd qi_1(3, 1);

        //目標の手先(x,y,z)を送り、各関節θ1 ~ θ3分動かすMatrixを得る
        qi_1 = robot.inverse_kinematics(robot.get_target_vec());

        //実際にθ1 ~ θ3動かし、その時の手先座標を得る
        std::vector<float> end_effector_vec = robot.direct_kinematics(qi_1);

        //手先座標x,y,zをsetする
        robot.set_Joint4_vec(end_effector_vec[0], end_effector_vec[1], end_effector_vec[2]);

    }

    return 0;
}
