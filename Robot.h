#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include <iostream>

    
class Point{
  public:
    Point() {}
    ~Point() {}
    float x;
    float y;
    float z;
};

class Robot
{
  public:
    Robot() {}
    ~Robot() {}

    std::vector<int> target_vec={1,1,1};


    std::vector<float> sig_0{0, 0, 0};


    //ある関節(θ1, θ2, θ3, θ4)を入れて、手先座標(x,y,z)を返す関数
    std::vector<float> direct_kinematics(std::vector<float> &joint_theta_vec);

    //ある手先座標(x,y,z)を入れて、すべての関節(θ1, θ2, θ3, θ4)を返す関数
    void inverse_kinematics(std::vector<float> &target_vec);

    //ある関節(θ1, θ2, θ3, θ4)を入れて、4×4の逆ヤコビ行列を返す関数
    std::vector<float> inv_Jacobian(std::vector<float> &target_vec);
};

//ある関節(θ1, θ2, θ3, θ4)を入れて、手先座標(x,y,z)を返す関数
std::vector<float> Robot::direct_kinematics(std::vector<float> &joint_theta_vec)
{
    std::vector<float> end_effector_vec;

    for (int i = 0; i < joint_theta_vec.size(); i++)
    {
        //ここで順運動学からpx, py, pzを求める
        float px = 0;
        float py = 1.0;
        float pz = 2.2;
        end_effector_vec.push_back(px);
        end_effector_vec.push_back(py);
        end_effector_vec.push_back(pz);
    }

    return end_effector_vec;
}

//ある手先座標(x,y,z)を入れて、すべての関節(θ1, θ2, θ3, θ4)を返す関数
void inverse_kinematics(std::vector<float> &target_vec)
{
    std::vector<int> target_vec = {1, 1, 1};


    Eigen::MatrixXd q_i_1(4, 1);

    q_i_1 = qi + k *inv_Jacobian;;
}

std::vector<float> inv_Jacobian(std::vector<float> &target_vec)
{
}

