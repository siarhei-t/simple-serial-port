/******************************************************************************
* File Name          : SerialDevice.cpp
* Author             : Siarhei Tatarchanka
* Version            : v 1.0
* Description        : 
*******************************************************************************/

#include "SerialDevice.hpp"

SerialDevice::SerialDevice()
{

}

SerialDevice::~SerialDevice()
{
    
}

int SerialDevice::CreatePortInstance(const std::string path)
{
    return 0;
}

int SerialDevice::DeletePortInstance(void)
{
    return 0;
}

void SerialDevice::GetListOfAvailableDevices(std::vector<std::string> &devices)
{

}

const SerialPort *SerialDevice::GetPointerToPort()
{
    return nullptr;
}
