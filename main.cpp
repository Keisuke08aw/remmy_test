#include <vector>
#include "Connection.h"
#include "Robot.h"

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

    Eigen::MatrixXd A(3, 1);

    std::vector<float> target_vec;
    target_vec.push_back(19.0);
    target_vec.push_back(0.0);
    target_vec.push_back(0.0);

    A = robot.inverse_kinematics(target_vec);
    return 0;
}
