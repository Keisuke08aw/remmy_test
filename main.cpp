#include <vector>
#include "Connection.h"
#include "Robot.h"
#include <iostream>
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
    printf("%d", robot.get_hoge());

    Eigen::MatrixXd qi_1(3, 1);

    std::vector<float> target_vec;
    target_vec.push_back(17.0);
    target_vec.push_back(0.0);
    target_vec.push_back(0.0);

    qi_1 = robot.inverse_kinematics(target_vec);

    // std::vector<float>型
    std::vector<float> vec_p = robot.direct_kinematics(qi_1);
    float px = vec_p[0];
    float py = vec_p[1];
    float pz = vec_p[2];

    robot.set_Joint4_vec(px, py, pz);
    Eigen::MatrixXd now_xyz = robot.get_Joint4_vec();

    printf("AAAOJOGAJOGAOJAOJOGA\r\n");
    std::cout << now_xyz << std::endl;

    return 0;
}
