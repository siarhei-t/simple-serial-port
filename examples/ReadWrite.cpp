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
    SerialDevice device;

    const SerialPort* p_port = device.GetPointerToPort();

    std::cout<<"compiler test"<<std::endl;
    return 0;
}
