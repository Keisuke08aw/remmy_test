#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include <math.h>
#include <iostream>

class Robot
{
  private:
    //x,y,z, θ
    static Eigen::MatrixXd joint1_vec;
    // static int joint1_vec;


  public:
    Robot() {}
    ~Robot() {}
    void setJoint(int num1, int num2, int num3){
        joint1_vec << num1, num2, num3;
    }

    // void setJoint(int num1)
    // {
    //     joint1_vec = num1;
    // }

    Eigen::MatrixXd getJoint()
    {
        return joint1_vec;
    }

    // int getJoint()
    // {
    //     return joint1_vec;
    // }
};
Eigen::MatrixXd Robot::joint1_vec(3, 1);
// Robot::joint1_vec << 1,2,3
// int Robot::joint1_vec(1,3);

int main()
{
    Robot robot;
    robot.setJoint(10, 11, 12);
    // robot.setJoint(10);
    Eigen::MatrixXd A = robot.getJoint();
    // int A = robot.getJoint();
    // printf("%d\n", robot.getJoint()[0]);
    std::cout << A(0) << std::endl;
    return 0;
}