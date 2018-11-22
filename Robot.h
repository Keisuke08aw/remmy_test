#include <vector>
#include <stdio.h>

class Robot
{
  public:
    Robot() {}
    ~Robot() {}

    //ある関節(θ1, θ2, θ3, θ4)を入れて、手先座標(x,y,z)を返す関数
    std::vector<float>direct_kinematics(std::vector<float> &theta_array);

    //ある手先座標(x,y,z)を入れて、すべての関節(θ1, θ2, θ3, θ4)を返す関数
    std::vector<float> inverse_kinematics(std::vector<float> &position_array);

    //ある関節(θ1, θ2, θ3, θ4)を入れて、4×4の逆ヤコビ行列を返す関数
    inverse_Jacobian(std::vector<unsigned char> &theta_array)
};

//ある関節(θ1, θ2, θ3, θ4)を入れて、手先座標(x,y,z)を返す関数
std::vector<float> Robot::direct_kinematics(std::vector<unsigned char> &theta_array)
{
  for (int i= 0; i < 4; i++){
    //ここで順運動学からpx, py, pzを求める
  }
}

//ある手先座標(x,y,z)を入れて、すべての関節(θ1, θ2, θ3, θ4)を返す関数
std::vector<float> Robot::inverse_kinematics()
{

}

