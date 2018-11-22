#include <vector>
#include "Connection.h"
#include "Eigen/Core"

#include <iostream>

void change_data(std::vector<float> &data_array)
{
    for (int i = 0; i < data_array.size(); i++)
    {
        printf("%f\n", data_array[i]);
    }
}

int main(int argc, char const *argv[])
{
    // Eigen::Matrix3d M;
    // M << 1, 2, 3,
    //     5, 6, 7,
    //     9, 10, 11;

    // Eigen::MatrixXd Jacobian(4, 4);
    // Jacobian << 1, 2, 3, 4,
    //             5, 6, 7, 8,
    //             9, 10, 11, 12;

    Eigen::MatrixXd Jacobian(4, 4);
    Jacobian << 1, 1, 1, 4,
        1, 1, 2, 4,
        1, 1, 3, 4,
        1, 1, 4, 4;

    /* code */
    std::vector<float> data;
    data.push_back(0);
    data.push_back(1);
    data.push_back(2);
    data.push_back(3);

    // change_data(data);
    // printf("%d\n", (int)data.size());
    // printf("%d", data[0]);
    // printf("%d", data[1]);
    // printf("%d", data[2]);
    // printf("%d", data[3]);

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
    A(0, 0) = 2;
    A(1, 1) = 5;

    // std::cout << X << std::endl;
    std::cout << Jacobian << std::endl;

    // printf("AAAAAAA");
    return 0;
}
