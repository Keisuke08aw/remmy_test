#include <vector>
#include <stdio.h>


// class Robot;
class Connection
{
  public:
    Connection() {}
    ~Connection() {}

    int open();
    int close();

    //send data to the robot. use explicit pointer convertion

    //ロボットに手先座標(x,y,z)を送る
    //data.size()==12より、float to byteでx,y,zを送る
    // int send(std::vector<unsigned char> &data);
    int send(int num);

    //receive state of the robot. record to data. use explicit pointer convertion
    //多分、ロボットの手先座標を受け取り、recordする

    // int receive(std::vector<unsigned char> &data);
};

