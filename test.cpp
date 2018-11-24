#include <vector>
#include "Connection.h"
#include "Eigen/Core"

#include <iostream>
class Robot{
  public:
    Robot() {}
    ~Robot() {}

    // private:
    //     char bb='a';
};

void ch_data(int &data)
{
    data +=1;
}

void change_data(std::vector<float> &data_array)
{
    for (int i = 0; i < data_array.size(); i++)
    {
        printf("%f\n", data_array[i]);
    }
}

int main(int argc, char const *argv[])
{
    // float x=1.0;
    // int i = 0x41;
    // float *num = &x;

    // std::vector<unsigned char> vec(4);
    // vec[0] = 1;
    // vec[1] = 2;
    // char x = -100;
    // unsigned char y;

    // y = (unsigned char)x;       // C static
    // y = *(unsigned char *)(&x); //
    // x = *(signed char *)(&y);

    // std::vector<unsigned char> vec1;
    // unsigned char  std::string str

    // vec1.push_back(1);
    // vec1.push_back(2);
    // vec1.push_back(3);
    // vec1.push_back(4);
    // vec1.push_back(3);
    // vec1.push_back(3);
    // vec1.push_back(3);

    int **data;



    // printf("%d")
    // vec1.push_back(a2);
    // vec1.push_back(a3);
    // vec1.push_back(a4);

    // printf("%d", (unsigned char)vec1.size());
    // std::cout << vec[0] << std::endl;


    // printf("%d", y);
    // printf("%d", x);

    // std::cout << ch+ch2 << std::endl;
    // std::cout << x << std::endl;

    // std::cout << &aaaa << std::endl;


    // std::vector<float> vec{1.0,2.0,3.0};

    // std::cout << &vec << std::endl;
    float rad1,rad2,rad3 = 1.6;

    Eigen::MatrixXd T_0_4(4, 4);

    Eigen::MatrixXd T_0_1(4, 4);
    Eigen::MatrixXd T_1_2(4, 4);
    Eigen::MatrixXd T_2_3(4, 4);
    Eigen::MatrixXd T_3_4(4, 4);

    T_0_1 << cos(rad1), -sin(rad1), 0, 0,
        sin(rad1), cos(rad1), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    T_1_2 << cos(rad2), sin(rad2), 0, 10,
        0, 0, -1, 0,
        -sin(rad2), cos(rad2), 0, 0,
        0, 0, 0, 1;

    T_2_3 << cos(rad3), -sin(rad3), 0, 5,
        sin(rad3), cos(rad3), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    T_3_4 << 1, 0, 0, 5,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4;
    Eigen::MatrixXd X(4, 4);

    // X = A*B;

    std::cout << T_0_4 << std::endl;

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            std::cout << T_0_4(i, j) << std::endl;
        }
    }
    // std::cout << T_0_4(0, 0) << std::endl;
    // std::cout << T_0_4(0, 0) << std::endl;

    // Eigen::MatrixXd Y = A + B;

    // Eigen::MatrixXd X = A *B;

    // /* code */
    // std::vector<float> data;
    // data.push_back(0);
    // data.push_back(1);
    // data.push_back(2);
    // data.push_back(3);

    // change_data(data);
    // printf("%d\n", (int)data.size());
    // printf("%d", data[0]);
    // printf("%d", data[1]);
    // printf("%d", data[2]);
    // printf("%d", data[3]);

    // Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
    // A(0, 0) = 2;
    // A(1, 1) = 5;
    // std::cout << A << std::endl;
    // std::cout << X << std::endl;
    // std::cout << Jacobian << std::endl;

    // std::cout << i << std::endl;

    // ch_data(Robot::age);
    // std::cout << Robot::age << std::endl;

    // printf("AAAAAAA");
    return 0;
}
