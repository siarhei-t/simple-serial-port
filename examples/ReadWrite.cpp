/******************************************************************************
* File Name          : ReadWrite.cpp
* Author             : Siarhei Tatarchanka
* Version            : v 1.0
* Description        : 
*******************************************************************************/

#include <iostream>

#include "SerialDevice.hpp"

int main(void)
{
    std::vector<std::string> devices;
    
    SerialDevice device;
    device.GetListOfAvailableDevices(devices);
    std::cout<<"Available ports :"<<std::endl;
    for(auto i = 0; i < devices.size();i++)
    {
        std::cout<<"dev "<<int(i)<<" : "<<devices[i]<<std::endl;
    }
    
    return 0;
}
