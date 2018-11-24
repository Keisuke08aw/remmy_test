#include <vector>
#include "Connection.h"
#include "Robot.h"

int main(int argc, char const *argv[])
{
    /* code */
    std::vector<unsigned char> data;

    Connection cn;
    int res1 = cn.open();
    if (res1 == 0)
    {
        int return1 = cn.send(data);
    }
    else
    {
        printf("Error");
    }

    


    return 0;
}
