#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include <math.h>

class Point
{
  public:
    Point() {}
    ~Point() {}
    double x;
    double y;
    double z;
};

class Robot
{
  std::vector<float> target_vec = {20, 0, 0};

  public:
    Robot() {}
    ~Robot() {}


    std::vector<double> joint0_vec{0, 0, 0, 0};
    std::vector<double> joint1_vec{0, 0, 0, 0};
    std::vector<double> joint2_vec{0, 0, 0, 0};
    std::vector<double> joint3_vec{0, 0, 0, 0};



    //ある関節(θ1, θ2, θ3, θ4)を入れて、手先座標(x,y,z)を返す関数
    std::vector<double> direct_kinematics(std::vector<double> &joint_theta_vec);

    //ある手先座標(x,y,z)を入れて、すべての関節(θ1, θ2, θ3, θ4)を返す関数
    std::vector<double> inverse_kinematics(std::vector<double> &target_vec);

        //ある関節(θ1, θ2, θ3, θ4)を入れて、4×4の逆ヤコビ行列を返す関数
    std::vector<double> inverse_kinematics(std::vector<double> &target_vec);
};

//ある関節(θ1, θ2, θ3, θ4)を入れて、手先座標(x,y,z)を返す関数
std::vector<double> Robot::direct_kinematics(std::vector<double> &joint_theta_vec)
{
    std::vector<double> end_effector_vec;

    for (int i = 0; i < joint_theta_vec.size(); i++)
    {
        //ここで順運動学からpx, py, pzを求める
        double px = 0;
        double py = 1.0;
        double pz = 2.2;
        end_effector_vec.push_back(px);
        end_effector_vec.push_back(py);
        end_effector_vec.push_back(pz);
    }

    return end_effector_vec;
}

//ある手先座標(x,y,z)を入れて、すべての関節(θ1, θ2, θ3, θ4)を返す関数
std::vector<double> inverse_kinematics(std::vector<double> &target_vec)
{
    double x1, x2, x3, x4 =0;
    Eigen::MatrixXd delta_r(4, 1);
    delta_r << sqrt(std::pow((target_vec[0] - x1), 2.0) + std::pow((target_vec[0] - y1), 2.0) + std::pow((target_vec[0] - z1), 2.0)),
        sqrt(std::pow((target_vec[0] - x1), 2.0) + std::pow((target_vec[0] - y1), 2.0) + std::pow((target_vec[0] - z1), 2.0)),
        sqrt(std::pow((target_vec[0] - x1), 2.0) + std::pow((target_vec[0] - y1), 2.0) + std::pow((target_vec[0] - z1), 2.0));

    std::vector<double> learning_rate = {0.01, 0.01, 0.01};
    Eigen::MatrixXd q_i_1(4, 1);

    q_i_1 = qi + k * inv_Jacobian;
    ;
}

std::vector<double> inv_Jacobian(std::vector<double> &target_vec)
{

}
