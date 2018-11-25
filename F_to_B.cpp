#include <stdio.h>
#include <stdint.h>
#include <vector>

//https://os.mbed.com/forum/helloworld/topic/2053/?page=1#comment-54720

std::vector<unsigned char> convert_Float_to_Byte(float num){
    std::vector<unsigned char> data;
    uint8_t bytes[sizeof(float)];
    *(float *)(bytes) = num; // convert float to bytes

    data.push_back(bytes[0]);
    data.push_back(bytes[1]);
    data.push_back(bytes[2]);
    data.push_back(bytes[3]);

    printf("data = [ %d, %d, %d, %d]\r\n", data[0], data[1], data[2], data[3]);
    return data;
}

float convert_Byte_to_Float(std::vector<unsigned char> &data){
    uint8_t bytes[sizeof(float)];
    for (int i = 0; i < 4; i++){
        bytes[i] = data[i];
        // printf(data)

    }

    float x_p = *(float *)(bytes); // convert bytes back to float

    printf("%f", x_p);
    return x_p;
}


int main()
{
    float num = 10.2;
    std::vector<unsigned char> result=convert_Float_to_Byte(num);
    convert_Byte_to_Float(result);

    // float x_p;
    // uint8_t bytes[sizeof(float)];

    // x_p = -20.0;

    // std::vector<unsigned char> data;

    // *(float *)(bytes) = x_p; // convert float to bytes

    // data.push_back(bytes[0]);
    // data.push_back(bytes[1]);
    // data.push_back(bytes[2]);
    // data.push_back(bytes[3]);

    // printf("////////////////Before///////////////////\r\n");
    // printf("x_p = %.6f\r\n", x_p);
    // printf("bytes = [ 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x ]\r\n", bytes[0], bytes[1], bytes[2], bytes[3]);
    // printf("data = [ %d, %d, %d, %d]\r\n", data[0], data[1], data[2], data[3]);

    // x_p = *(float *)(bytes); // convert bytes back to float
    // printf("////////////////After///////////////////\r\n");
    // printf("x_p = %.6f\r\n", x_p);

    return 0;
}