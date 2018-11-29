#include <vector>
#include "Connection.h"
#include "Robot.h"
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES

std::vector<unsigned char> convert_Float_to_Byte(float x, float y, float z)
{
    std::vector<unsigned char> data;
    uint8_t bytes1[sizeof(float)];
    uint8_t bytes2[sizeof(float)];
    uint8_t bytes3[sizeof(float)];

    *(float *)(bytes1) = x; // convert float to bytes
    *(float *)(bytes2) = y; // convert float to bytes
    *(float *)(bytes3) = z; // convert float to bytes

    for (int i = 0; i < 4; i++)
    {
        data.push_back(bytes1[i]);
    }

    for (int i = 0; i < 4; i++)
    {
        data.push_back(bytes2[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        data.push_back(bytes3[i]);
    }

    return data;
}

int main(int argc, char const *argv[])
{
    Robot robot;
    Connection connection;
    //input.inをすべて読み込み、connectionのメンバ変数にセット

    int result1 = connection.open();
    if (result1 == false)
    {
        printf("FILE READ ERROR");
        std::exit(1);
    }

    //get all points
    std::vector<float> trajectory_vec_list = connection.get_trajectory();

    std::vector<unsigned char> target_p;
    std::vector<unsigned char> byte_target_p;
    std::vector<unsigned char> target_y;
    std::vector<unsigned char> target_z;

    for (int i = 0; i < trajectory_vec_list.size(); i += 3)
    {

        //trajectory_vec_list ={20.0, 0.0, 0.0, 17.0, 0.0, ....}
        float x = trajectory_vec_list[i];
        float y = trajectory_vec_list[i + 1];
        float z = trajectory_vec_list[i + 2];

        byte_target_p = convert_Float_to_Byte(x, y, z);

        // set first joint angles
        float rad1 = -(float)M_PI;
        float rad2 = -(float)M_PI / 4;
        float rad3 = (float)M_PI / 2;

        //set initial position of the robot arm
        robot.set_Joint1Angle(rad1);
        robot.set_Joint2Angle(rad2);
        robot.set_Joint3Angle(rad3);

        int result2 = connection.send(byte_target_p);

        if (result2 == false)
        {
            printf("CONNECTION SEND ERROR HAPPENS");
            std::exit(1);
        }
        byte_target_p.clear();
    }
    return 0;
}
