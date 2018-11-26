#include "Connection.h"
#include "Robot.h"

std::vector<float> Connection::get_trajectory()
{
    return trajectory_vec_list;
}
void Connection::set_trajectory(std::vector<float> vec_list)
{
    trajectory_vec_list = vec_list;
}

int Connection::open()
{
    FILE *fp; // FILE型構造体
    char fname[] = "input.in";
    fp = fopen(fname, "r"); // ファイルを開く。失敗するとNULLを返す。
    if (fp == NULL)
    {
        printf("%s file not open!\n", fname);
        return -1;
    }
    else
    {
        std::vector<float> vec_list;
        float data;
        int counter = 0;
        while (fscanf(fp, "%f", &data) != EOF)
        {
            if (counter != 3 && counter != 7 && counter != 11 && counter != 15 && counter != 19 && counter != 23 && counter != 27)
            {
                vec_list.push_back(data);
            }
            counter += 1;
        }
        Connection connection;
        connection.set_trajectory(vec_list);
        return 0;
    }
}

int Connection::close()
{
    FILE *fp; // FILE型構造体
    char fname[] = "input.in";

    int ret = fclose(fp);
    if (ret == EOF)
    {
        printf("ファイルクローズに失敗しました。\n");
        return -1;
    }
    else
    {
        printf("ファイルをクローズしました。\n");
        return 0;
    }
};

// dataの大きさは12、4,4,4
int Connection::send(std::vector<unsigned char> &data)
{
    Robot robot;
    int result;
    //ロボットに手先座標(x, y, z)をコントロールシグナルとして送る
    result = robot.inverse_kinematics(data);
    return result;
};

// int Connection::receive(std::vector<unsigned char> &data){

// };

std::vector<float> Connection::trajectory_vec_list;
