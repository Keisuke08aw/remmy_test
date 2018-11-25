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


  public:
    Robot() {}
    ~Robot() {}
    void setJoint(int num1, int num2, int num3){
        joint1_vec << num1, num2, num3;
    }
    Eigen::MatrixXd getJoint()
    {
        return joint1_vec;
    }


};
Eigen::MatrixXd Robot::joint1_vec(3, 1);

int main()
{
    Robot robot;
    robot.setJoint(10, 11, 12);
    // robot.setJoint(10);
    Eigen::MatrixXd A = robot.getJoint();
    std::cout << A(0) << std::endl;
    return 0;
}