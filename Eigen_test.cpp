#include <vector>
#include "Connection.h"
#include "Eigen/Core"
// #include "stdio.h"
#include <iostream>
#include <Eigen/LU>

std::vector<double> direct_kinematics(){
    std::vector<double> vec={1.0,2.0,3.0};
    return vec;
};

int main(int argc, char const *argv[])
{
    std::vector<double> v1;
    v1 = direct_kinematics();
    printf("%f\n", v1[0]);
    printf("%f\n", v1[1]);
    printf("%f\n", v1[2]);

    // std::cout << v1 << std::endl;
 
    return 0;
}
