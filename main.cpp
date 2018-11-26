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
    printf("First target z %.2f\r\n", z);
    printf("/////////////////////////////////\r\n");


    std::vector<unsigned char> target_p;
    std::vector<unsigned char> target_x = convert_Float_to_Byte(x);
    std::vector<unsigned char> target_y = convert_Float_to_Byte(y);
    std::vector<unsigned char> target_z = convert_Float_to_Byte(z);

    target_p.insert(target_p.end(), target_x.begin(), target_x.end());
    target_p.insert(target_p.end(), target_y.begin(), target_y.end());
    target_p.insert(target_p.end(), target_z.begin(), target_z.end());

    // printf("target bytes0 %d\r\n", target_p[0]);
    // printf("target bytes1 %d\r\n", target_p[1]);
    // printf("target bytes2 %d\r\n", target_p[2]);
    // printf("target bytes3 %d\r\n\r\n", target_p[3]);

    Robot robot;

    // //関節の初期値を適当に決める
    float rad1 = M_PI/4;
    float rad2 = M_PI/2;
    float rad3 = M_PI/2;

    robot.set_Joint1Angle(rad1);
    robot.set_Joint2Angle(rad2);
    robot.set_Joint3Angle(rad3);

    int result;
    result = connection.send(target_p);


    return 0;
}
