#include <vector>
#include "Connection.h"

std::vector<float> func(std::vector<float> data)
{
    std::vector<float> vec2;
    for (int i = 0; i < 3; i++)
    {
        vec2.push_back(data[i] + 2);
    }
    return vec2;
}

int main(int argc, char const *argv[])
{
    std::vector<float> data;
    data.push_back(17.0);
    data.push_back(0.0);
    data.push_back(0.0);

    std::vector<float> vec2;

    vec2 = func(data);

    for (int i = 0; i < 3; i++)
    {
        printf("%f\r\n", vec2[i]);
    }

    return 0;
}