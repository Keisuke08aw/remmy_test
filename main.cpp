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

    printf("data = [ %d, %d, %d, %d]\r\n", data[0], data[1], data[2], data[3]);

    return data;
}

float convert_Byte_to_Float(std::vector<unsigned char> &data)
{
    uint8_t bytes[sizeof(float)];
    for (int i = 0; i < 4; i++)
    {
        bytes[i] = data[i];
        // printf(data)
    }
    float x_p = *(float *)(bytes); // convert bytes back to float

    // printf("float %f\r\n", x_p);
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

    printf("%f\r\n", x);
    printf("%f\r\n", y);
    printf("%f\r\n", z);

    std::vector<unsigned char> target_p;
    std::vector<unsigned char> target_x = convert_Float_to_Byte(x);
    std::vector<unsigned char> target_y = convert_Float_to_Byte(y);
    std::vector<unsigned char> target_z = convert_Float_to_Byte(z);

    target_p.insert(target_p.end(), target_x.begin(), target_x.end());
    target_p.insert(target_p.end(), target_y.begin(), target_y.end());
    target_p.insert(target_p.end(), target_z.begin(), target_z.end());

    printf("%d\r\n", target_p[0]);
    printf("%d\r\n", target_p[1]);
    printf("%d\r\n", target_p[2]);
    printf("%d\r\n", target_p[3]);

    printf("%lu\r\n", target_p.size());

    //std::vector<unsigned char> &data
    //１点のみ送るだから、data.seiz() == 12;
    //target_p = {0, 0, 160, 65, 0, 0, 0, 0...}
    connection.send(target_p);

    Robot robot;

    std::vector<float> target_vec;
    target_vec.push_back(17.0);
    target_vec.push_back(0.0);
    target_vec.push_back(0.0);

    Eigen::MatrixXd init_q(3, 1);

    //初期値を適当に決める
    init_q << 0.0, M_PI / 2, -M_PI / 2;

    //適当に決めたときの手先座標x,y,zを得る
    std::vector<float> vec_p = robot.direct_kinematics(init_q);

    float px = vec_p[0];
    float py = vec_p[1];
    float pz = vec_p[2];

    robot.set_Joint4_vec(px, py, pz);
    // qi_1 = robot.inverse_kinematics(target_vec);

    // while ((float)robot.delta(target_vec) != 0.0)
    // {
    //     float delta_xyz =robot.delta(target_vec);
    //     // printf("%f\r\n", delta_xyz);

    //     // printf("AAAAAA");
    //     Eigen::MatrixXd qi_1(3, 1);

    //     //θ1 ~ θ3を返す
    //     qi_1 = robot.inverse_kinematics(target_vec);

    //     //その時の、手先座標x,y,zを得る
    //     std::vector<float> vec_p = robot.direct_kinematics(qi_1);
    //     float px = vec_p[0];
    //     float py = vec_p[1];
    //     float pz = vec_p[2];

    //     printf("%f %f %f\r\n", px, py, pz);
    //     // printf("%f\r\n", py);
    //     // printf("%f\r\n", pz);

    //     //手先座標をセットする
    //     robot.set_Joint4_vec(px, py, pz);
    // }

    return 0;
}
