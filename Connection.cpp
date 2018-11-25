#include "Connection.h"
#include "Robot.h"

// int Connection::open()
// {
//     fp = fopen(fname, "r"); // ファイルを開く。失敗するとNULLを返す。
//     if (fp == NULL)
//     {
//         printf("%s file not open!\n", fname);
//         return -1;
//     }
//     else
//     {
//         printf("%s file opened!\n", fname);
//         return 0;
//     }
// }

// int Connection::close()
// {
//     int ret = fclose(fp);
//     int result;
//     if (ret == EOF)
//     {
//         printf("ファイルクローズに失敗しました。\n");
//         return -1;
//     }
//     else
//     {
//         printf("ファイルをクローズしました。\n");
//         return 0;
//     }
// };

// int Connection::send(std::vector<unsigned char> &data){
//     Robot robot;
//     robot.set_data(data);
// };

int Connection::send(int num){
    Robot robot;
    robot.set_hoge(num);
    printf("%d", robot.get_hoge());
    return 1;
}

// int Connection::receive(std::vector<unsigned char> &data){

// };