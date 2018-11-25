#include <vector>
#include <stdio.h>
#include "Eigen/Core"
#include <math.h>

class Robot
{
private:
  //θ
  static float joint1_angle;
  static float joint2_angle;
  static float joint3_angle;
  //x, y, z
  static Eigen::MatrixXd joint4_vec;

  static int hoge;

public:
  Robot() {}
  float get_Joint1Angle();
  float get_Joint2Angle();
  float get_Joint3Angle();
  Eigen::MatrixXd get_Joint4_vec();

  void set_Joint1Angle(float radian);
  void set_Joint2Angle(float radian);
  void set_Joint3Angle(float radian);
  void set_Joint4_vec(float x, float y, float z);

  //ある関節(θ1, θ2, θ3)を入れて、今の手先座標(x,y,z)を返す関数
  std::vector<float> direct_kinematics(Eigen::MatrixXd vec_joint_angle);

  //ある手先座標(x,y,z)を入れて、次の関節の角度(θ1, θ2, θ3, θ4)を返す関数
  Eigen::MatrixXd inverse_kinematics(std::vector<float> target_vec);

  float delta(std::vector<float> target_vec);

      int get_hoge();
  void set_hoge(int num);

  ~Robot() {}
};
