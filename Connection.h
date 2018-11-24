#include <vector>
#include <stdio.h>

FILE *fp; // FILE型構造体
char fname[] = "test.txt";

class Connection
{
  public:
    Connection() {}
    ~Connection() {}

    int open();
    int close();

    //send data to the robot. use explicit pointer convertion

    //ロボットに手先座標(x,y,z)を送る
    //vector(0~255, 0~255, 0~255)で送る
    //その結果をintで返す
    //data.size()==12らしい、だから多分θ1~θ3だけ？
    int send(std::vector<unsigned char> &data);

    //receive state of the robot. record to data. use explicit pointer convertion

    //ロボットの関節の位置(x,y,z)の情報を受け取り、その値を記録する
    //手先座標だけ？それともすべての関節？
    //(x,y,z)をvector(0~255, 0~255, 0~255)で順キネの値を受け取る
    int receive(std::vector<unsigned char> &data);
};

int Connection::open()
{
    fp = fopen(fname, "r"); // ファイルを開く。失敗するとNULLを返す。
    if (fp == NULL)
    {
        printf("%s file not open!\n", fname);
        return -1;
    }
    else
    {
        printf("%s file opened!\n", fname);
        return 0;
    }
}

int Connection::close()
{
    int ret = fclose(fp);
    int result;
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
}
