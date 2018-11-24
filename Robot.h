#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include <math.h>

class Robot
{
  private:
    //x,y,z, θ
    static Eigen::MatrixXd joint1_vec;
    static Eigen::MatrixXd joint2_vec(1, 4);
    static Eigen::MatrixXd joint3_vec(1, 4);

  public:
    Robot() {}

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
    ~Robot() {}
};

