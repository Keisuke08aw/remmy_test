#include "Connection.h"
#include "Robot.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib> // EXIT_FAILURE のため

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
    Connection connection;
    char fname[] = "input.in";
    file_pointer = fopen(fname, "r"); //
    if (file_pointer == NULL)
    {
        printf("%s FILE cannot open!\n", fname);
        return false;
    }
    else
    {
        std::vector<float> vec_list;
        float data;
        int counter = 0;
        while (fscanf(file_pointer, "%f", &data) != EOF)
        {
            if (counter != 3 && counter != 7 && counter != 11 && counter != 15 && counter != 19 && counter != 23 && counter != 27)
            {
                vec_list.push_back(data);
            }
            counter += 1;
        }

        connection.set_trajectory(vec_list);
        return true;
    }
}

int Connection::close()
{
    int result = fclose(file_pointer);
    if (result == EOF)
    {
        printf("FILE CLOSE OKAY。\n");
        return true;
    }
    else
    {
        printf("FILE CLOSE ERROR。\n");
        return false;
    }
};

//
int Connection::send(std::vector<unsigned char> &data)
{
    Robot robot;
    int result;
    //send target point x,y,z data 12size
    result = robot.inverse_kinematics(data);
    return result;
};

int Connection::receive(std::vector<unsigned char> &data)
{
    // convert bytes back to float
    std::fstream fs;
    int result = true;
    fs.open("output.txt", std::ios::app);
    if (!fs.is_open())
    {
        result = false;
        return result;
    }

    uint8_t bytes1[sizeof(float)];
    uint8_t bytes2[sizeof(float)];
    uint8_t bytes3[sizeof(float)];

    for (int i = 0; i < 4; i++)
    {
        bytes1[i] = data[i];
        bytes2[i] = data[i + 4];
        bytes3[i] = data[i + 8];
    }
    //target x y z
    float x_p = *(float *)(bytes1); // convert bytes back to float
    float y_p = *(float *)(bytes2); // convert bytes back to float
    float z_p = *(float *)(bytes3); // convert bytes back to float

    fs << "x:";             //
    fs << x_p;              //
    fs << " ";              //
    fs << "y:";             //
    fs << y_p;              //
    fs << " ";              //
    fs << "z:";             //
    fs << z_p << std::endl; //

    fs.close();

    return result;
};

std::vector<float> Connection::trajectory_vec_list;
FILE *Connection::file_pointer; // FILE型構造体
