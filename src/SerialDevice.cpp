/******************************************************************************
* File Name          : SerialDevice.cpp
* Author             : Siarhei Tatarchanka
* Version            : v 1.0
* Description        : 
*******************************************************************************/

#include <iostream>

#include "SerialDevice.hpp"

#if defined(TARGET_LINUX)
#include <sys/types.h>
#include <dirent.h>
#include "SerialPortLinux.hpp"

static SerialPortLinux* pActualPort;
#endif //TARGET_LINUX


SerialDevice::SerialDevice()
{
    this->port = nullptr;
}

SerialDevice::~SerialDevice()
{
    if(this->port != nullptr)
    {
        if(this->port->GetPortState() != PortState::STATE_CLOSE)
        {
            this->port->Close(); 
        }
        delete this->port;
        this->port = nullptr;
    }    
}

void SerialDevice::GetListOfAvailableDevices(std::vector<std::string> &devices)
{
    devices.clear();

    #if defined(TARGET_LINUX)
    const char path[] = {"/dev/"};
    static const std::string dev_template[] = {"ttyUSB","ttyACM"};
    dirent *dp;
    DIR *dirp;
    dirp = opendir(path);
    while ((dp = readdir(dirp)) != NULL)
    {
        size_t      tab_size = sizeof(dev_template)/sizeof(std::string);
        std::string device   = dp->d_name;
        
        for(size_t i = 0; i < tab_size; i++)
        {
            int result = device.compare(0, dev_template[i].size(), dev_template[i]);
            if(result == 0)
            {
                devices.push_back(device);
            }else{continue;}
        }
    }
    (void)closedir(dirp);
    #endif //TARGET_LINUX
}

SerialPort *SerialDevice::GetPointerToPort()
{
    return this->port;
}

int SerialDevice::CreatePortInstance(const std::string path)
{
    int stat = -1;
    #if defined(TARGET_LINUX)
    pActualPort = new SerialPortLinux(path);
    if(pActualPort->GetPortState() == PortState::STATE_OPEN)
    {
        //this->port = (SerialPort*)(actualPort);
        this->port = dynamic_cast<SerialPort*>(pActualPort);
        if(this->port){stat = 0;}
    }
    #endif
    return stat;
}

int SerialDevice::DeletePortInstance(void)
{
    return 0;
}


