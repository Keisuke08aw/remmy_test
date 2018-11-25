#include <stdio.h>
#include <stdint.h>

int main()
{
    union {
        uint8_t bytes[4];
        float f;
    } fun1 = {.bytes = {0x41, 0xd7, 0xd1, 0xe1}},
    fun2 = {.bytes = {0xe1, 0xd1, 0xd7, 0x41}};

    printf("%f\n%f\n", fun1.f, fun2.f);

    return 0;
}