#include <vector>
#include <stdio.h>

// class Robot;
class Connection
{
  private:
    static std::vector<float> trajectory_vec_list;

  public:
    Connection() {}
    ~Connection() {}

    int open();
    int close();

    std::vector<float> get_trajectory();
    void set_trajectory(std::vector<float> vec_list);


    //send data to the robot. use explicit pointer convertion

    //ロボットに手先座標(x,y,z)を送る
    //data.size()==12より、float to byteでx,y,zを送る
    int send(std::vector<unsigned char> &data);

    //receive state of the robot. record to data. use explicit pointer convertion
    //多分、ロボットの手先座標を受け取り、recordする
    int receive(std::vector<unsigned char> &data);
};
