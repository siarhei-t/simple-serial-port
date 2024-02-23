/**
 * @file read_write.hpp
 *
 * @brief 
 *
 * @author Siarhei Tatarchanka
 *
 */

#include <iostream>
#include "serial_port.hpp"

int main(void)
{
    SerialDevice sys_serial;
    auto actual_list = sys_serial.getListOfAvailableDevices();
    for(auto i = 0; i < actual_list.size(); ++i)
    {
        std::cout<<" available device : "<<actual_list[i]<<" | index : "<<i<<"\n";
    }
    return 0;
}
