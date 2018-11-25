#include "Connection.h"
#include "Robot.h"

std::vector<float> Connection::get_trajectory(){
    return trajectory_vec_list;
}
void Connection::set_trajectory(std::vector<float> vec_list){
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
        int counter =0;
        while (fscanf(fp, "%f", &data) != EOF)
        {
            vec_list.push_back(data);
        //     printf("%f\n", data);

        //     if(counter % 4 != 0){
        //         vec_list.push_back(data);
        //     }
        //     counter +=1;
        // }
        // for (int i = 0; i < 10; i++){
        //     printf("%f", vec_list[i]);
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


int Connection::send(std::vector<unsigned char> &data){
    Robot robot;
    robot.set_Target_vec(data);
};

int Connection::receive(std::vector<unsigned char> &data){

};

std::vector<float> Connection::trajectory_vec_list;

