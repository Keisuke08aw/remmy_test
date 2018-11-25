#include <vector>
#include <stdio.h>
#include <math.h>
#include <iostream>


int main()
{
    std::vector<float> point;
    point.push_back(20.0);
    point.push_back(0.0);
    point.push_back(0.0);

    std::vector<unsigned char> data;
    unsigned char const *result = reinterpret_cast<unsigned char const *>(&point[0]);

    for (std::size_t i = 0; i != sizeof(float); ++i)
    {
        unsigned int value = result[i];
    
        // value = pre+value;
        printf("%02X\n", value);
        // std::printf("0x%02X\n", result[i]);


    }
    // std::cout << data << std::endl;

    // unsigned char c= 128;
    // float const *q = reinterpret_cast<float const *>(c);
    // printf("%f\n", q);

    // for (std::size_t i = 0; i != sizeof(unsigned char); ++i)
    // {
    //     printf("%02f\n", q[i]);
    //     // std::printf("The byte #%zu is 0x%02X\n", i, p[i]);
    // }

    return 0;
}