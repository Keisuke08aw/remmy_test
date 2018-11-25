#include <vector>
#include "Connection.h"
#include "Robot.h"
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES

int main(int argc, char const *argv[])
{
    /* code */
    // std::vector<unsigned char> data;

    // Connection cn;
    // int res1 = cn.open();
    // if (res1 == 0)
    // {
    //     int return1 = cn.send(data);
    // }
    // else
    // {
    //     printf("Error");
    // }
    Connection connection;
    connection.send(10);

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

    while ((float)robot.delta(target_vec) != 0.0)
    {
        float delta_xyz =robot.delta(target_vec);
        // printf("%f\r\n", delta_xyz);

        // printf("AAAAAA");
        Eigen::MatrixXd qi_1(3, 1);

        //θ1 ~ θ3を返す
        qi_1 = robot.inverse_kinematics(target_vec);

        //その時の、手先座標x,y,zを得る
        std::vector<float> vec_p = robot.direct_kinematics(qi_1);
        float px = vec_p[0];
        float py = vec_p[1];
        float pz = vec_p[2];

        printf("%f %f %f\r\n", px, py, pz);
        // printf("%f\r\n", py);
        // printf("%f\r\n", pz);

        //手先座標をセットする
        robot.set_Joint4_vec(px, py, pz);
    }
    return 0;
}
