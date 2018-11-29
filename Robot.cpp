//
// Created by Keisuke Umezawa on 2018/11/27.
//

#include "Eigen/LU"
#include <math.h>
#include "Robot.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <unistd.h>
#include "Connection.h"



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

std::vector<float> Robot::get_EndEffector_vec()
{
    return end_effector_vec;
}

void Robot::set_Joint1Angle(float radian)
{
    if(radian < -M_PI){
        radian = -M_PI;
    }
    if(M_PI < radian){
        radian = M_PI;
    }
    joint1_angle = radian;
}

void Robot::set_Joint2Angle(float radian)
{
    if(radian < -M_PI/2){
        radian = -M_PI/2;
    }
    if(M_PI/2 < radian){
        radian = M_PI/2;
    }
    joint2_angle = radian;
}

void Robot::set_Joint3Angle(float radian)
{
    float singular_point = 0.0;
    if(radian < -M_PI){
        radian = -M_PI;
    }
    if(M_PI < radian){
        radian = M_PI;
    }
    if(radian == singular_point){
        radian = M_PI/12;
    }

    joint3_angle = radian;
}
void Robot::set_EndEffector_vec(std::vector<float> end_vec)
{
    end_effector_vec = end_vec;

}

float Robot::joint1_angle = -M_PI / 2;
float Robot::joint2_angle = 0.0;
float Robot::joint3_angle = 0.0;

std::vector<float> Robot::target_vec;
std::vector<float> Robot::end_effector_vec(3);

//input joint angle (θ1, θ2, θ3)、output end_effector_vec (x,y,z)
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

std::vector<float> Robot::convert_Byte_to_Float(std::vector<unsigned char> &data)
{
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

    std::vector<float> byte_to_float_vec;
    byte_to_float_vec.push_back(x_p);
    byte_to_float_vec.push_back(y_p);
    byte_to_float_vec.push_back(z_p);
    return byte_to_float_vec;
}

std::vector<unsigned char> Robot::convert_Float_to_Byte(float x, float y, float z)
{
    std::vector<unsigned char> data;
    uint8_t bytes1[12];
    uint8_t bytes2[sizeof(float)];
    uint8_t bytes3[sizeof(float)];

    *(float *)(bytes1) = x; // convert float to bytes
    *(float *)(bytes2) = y; // convert float to bytes
    *(float *)(bytes3) = z; // convert float to bytes

    for(int i =0; i<4; i++){
        data.push_back(bytes1[i]);
        data.push_back(bytes2[i]);
        data.push_back(bytes3[i]);
    }

    return data;
}


//input end_effector_vec (x,y,z)、return joint angles (θ1, θ2, θ3)
int Robot::inverse_kinematics(std::vector<unsigned char> &data)
{
    Connection connection;
    Robot robot;
    // convert bytes back to float
    std::vector<float> byte_to_float_vec = robot.convert_Byte_to_Float(data);
    //target x y z
    float target_x = byte_to_float_vec[0];
    float target_y = byte_to_float_vec[1];
    float target_z = byte_to_float_vec[2];
    //end_effector
    std::vector<float> end_effector_vec;
    std::vector<unsigned char> byte_end_effector_vec;
    //joint angles
    float rad1;
    float rad2;
    float rad3;

    Eigen::MatrixXd qi_1(3, 1);
    Eigen::MatrixXd qi(3, 1);
    Eigen::MatrixXd learning_rate(3, 3);
    Eigen::MatrixXd inv_Jacobian(3, 3);
    Eigen::MatrixXd Jacobian(3, 3);
    Eigen::MatrixXd ri(3, 1);
    Eigen::MatrixXd rad(3, 1);


    float last_delta_xyz=0;
    float delta_xyz;
    float delta_x;
    float delta_y;
    float delta_z;
    float k = 2.0;
    learning_rate << k, 0, 0,
               0, k, 0,
                0, 0, k;
    int result;
    int recieve_result;

    try
    {
        while (true)
        {
//            printf("//////////////////////////\n");
            rad1 = robot.get_Joint1Angle();
            rad2 = robot.get_Joint2Angle();
            rad3 = robot.get_Joint3Angle();

            qi << rad1, rad2, rad3;

            printf("rad1 %.2f\n", rad1);
            printf("rad2 %.2f\n", rad2);
            printf("rad3 %.2f\n", rad3);

            end_effector_vec = robot.direct_kinematics(qi);

            printf("End x %.2f\n", end_effector_vec[0]);
            printf("End y %.2f\n", end_effector_vec[1]);
            printf("End z %.2f\n", end_effector_vec[2]);

            // convert float to bytes
            byte_end_effector_vec = convert_Float_to_Byte(end_effector_vec[0], end_effector_vec[1],end_effector_vec[2]);


            recieve_result = connection.receive(byte_end_effector_vec);
            if (recieve_result == false){
                printf("DATA RECEIVE ERRORS");
                exit(1);
            }

            byte_end_effector_vec.clear();

            //calculate the distance between end_effector to target
            delta_xyz = (float)sqrt(pow((target_x - end_effector_vec[0]), 2) + pow((target_y - end_effector_vec[1]), 2) + pow((target_z - end_effector_vec[2]), 2));

            //if the delta is the same value, this tries to escape the situation by randomly change the joint angles
            if(last_delta_xyz == delta_xyz){
                int delta_rad1 = 2 + (int)(rand()*(6-2+1.0)/(1.0+RAND_MAX));
                int delta_rad2 = 2 + (int)(rand()*(6-2+1.0)/(1.0+RAND_MAX));
                int delta_rad3 = 2 + (int)(rand()*(6-2+1.0)/(1.0+RAND_MAX));

                rad1 = rad1 + M_PI/delta_rad1;
                rad2 = rad2 + M_PI/delta_rad2;
                rad3 = rad3 + M_PI/delta_rad3;

                robot.set_Joint1Angle(rad1);
                robot.set_Joint2Angle(rad2);
                robot.set_Joint3Angle(rad3);

                qi << rad1, rad2, rad3;

                end_effector_vec = robot.direct_kinematics(qi);
            }

            printf("delta_xyz  %.5f\n", delta_xyz);

            //if the distance is under 0.1, the calculation is stopped
            if (delta_xyz <0.1)
            {
                qi << rad1, rad2, rad3;
                std::vector<float> nearest_end_vec = robot.direct_kinematics(qi);
                robot.set_EndEffector_vec(nearest_end_vec);
                printf("DELTA IS VERY SMALL\n");
                result = true;
                break;
            }

            //EACH Jacobian value is calculated below
            Jacobian(0, 0) = -sin(rad1) * cos(rad2) * (5 * cos(rad3) + 5) + (-sin(rad1)) * (5 * sin(rad2) * sin(rad3) + 10);
            Jacobian(0, 1) = cos(rad1) * (-sin(rad2)) * (5 * cos(rad3) + 5) + 0;
            Jacobian(0, 2) = cos(rad1) * cos(rad2) * 5 * (-sin(rad3)) + cos(rad1) * 5 * sin(rad2) * cos(rad3);
            Jacobian(1, 0) = 5 * cos(rad1) * cos(rad2) * (cos(rad3) + 1) + 5 * cos(rad1) * (sin(rad2) * sin(rad3) + 2);
            Jacobian(1, 1) = 5 * sin(rad1) * (-sin(rad2)) * (cos(rad3) + 1) + 5 * sin(rad1) * cos(rad2) * sin(rad3);
            Jacobian(1, 2) = 5 * sin(rad1) * cos(rad2) * (-sin(rad3)) + 5 * sin(rad1) * sin(rad2) * cos(rad3);
            Jacobian(2, 0) = 0.0;
            Jacobian(2, 1) = -5 * cos(rad2) * (cos(rad3) + 1) - 5 * cos(rad2) * sin(rad3);
            Jacobian(2, 2) = -5 * sin(rad2) * (-sin(rad3)) + 5 * cos(rad2) * cos(rad3);

            delta_x = (float)sqrt(pow((target_x - end_effector_vec[0]), 2));
            delta_y = (float)sqrt(pow((target_y - end_effector_vec[1]), 2));
            delta_z = (float)sqrt(pow((target_z - end_effector_vec[2]), 2));

            qi << rad1, rad2, rad3;
            ri << delta_x, delta_y, delta_z;
            inv_Jacobian = Jacobian.inverse();

            std::cout << inv_Jacobian << std::endl;

            //by this repeatedly, the end_effector is coming to the target position
            qi_1 = qi - learning_rate * inv_Jacobian * ri;

            //set joints θ1~θ3
            robot.set_Joint1Angle(qi_1(0));
            robot.set_Joint2Angle(qi_1(1));
            robot.set_Joint3Angle(qi_1(2));

            end_effector_vec = robot.direct_kinematics(qi_1);
            robot.set_EndEffector_vec(end_effector_vec);

            last_delta_xyz = delta_xyz;
            printf("//////////////////////////\n");
//            sleep(1);
        }
    }

    catch (std::exception e)
    {
        printf("ERROR HAPPENS\r\n");

        result = false;
    }
    connection.close();
    return result;
}
