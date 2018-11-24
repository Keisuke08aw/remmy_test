#include <vector>
#include "Connection.h"
#include "Eigen/Core"
// #include "stdio.h"
#include <iostream>
#include "Eigen/LU"

template <class T>
void print(const T &value)
{
    std::cout << value << std::endl;
}

int main(int argc, char const *argv[])
{
    Eigen::Matrix3f A;
    A << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    Eigen::MatrixXd m(3, 3);
    m << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;

    std::cout << A << std::endl;
    std::cout << A.inverse() << std::endl;

    // Eigen::Matrix3d n;
    // n = m.inverse();

    // for (int i = 0; i < 3;i++){
    //     printf("%f\n", m(i,0));
    // }
    return 0;
}
