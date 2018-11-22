#include <vector>
#include "Connection.h"

int main(int argc, char const *argv[])
{
    /* code */
    std::vector<unsigned char> data;
    data.push_back(0);
    data.push_back(1);
    data.push_back(2);
    data.push_back(3);

    printf("%d\n", (int)data.size());
    // printf("%d", data[0]);
    // printf("%d", data[1]);
    // printf("%d", data[2]);
    // printf("%d", data[3]);

    // printf("AAAAAAA");
    return 0;
}
