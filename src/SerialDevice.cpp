/******************************************************************************
* File Name          : SerialDevice.cpp
* Author             : Siarhei Tatarchanka
* Version            : v 1.0
* Description        : 
*******************************************************************************/

#include <iostream>
#include "SerialDevice.hpp"

#if defined(PLATFORM_LINUX) && !defined(PLATFORM_WINDOWS)
#include <sys/types.h>
#include <dirent.h>
#include "SerialPortLinux.hpp"
static SerialPortLinux* pActualPort;
#elif defined(PLATFORM_WINDOWS) && !defined(PLATFORM_LINUX)
#include <windows.h>
#include "SerialPortWindows.hpp"
static SerialPortWindows* pActualPort;
#else
static SerialPort* pActualPort;
#endif

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

    #if defined(PLATFORM_WINDOWS) && !defined(PLATFORM_LINUX)
    char* dev_path = new char[256];

    for (auto i = 0; i < 255; i++)
    {
        std::string device = "COM" + std::to_string(i);
        DWORD result = QueryDosDevice(device.c_str(),dev_path, 256);
        if (result != 0)
        {
            devices.push_back(device);
        }else{continue;}
    }
    delete[] dev_path;
    #endif

    #if defined(PLATFORM_LINUX) && !defined(PLATFORM_WINDOWS) 
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
    #endif

}

SerialPort *SerialDevice::GetPointerToPort()
{
    return this->port;
}

int SerialDevice::CreatePortInstance(const std::string path,const PortConfig& config)
{
    int stat = -1;
    #if defined(PLATFORM_LINUX) && !defined(PLATFORM_WINDOWS) 
    pActualPort = new SerialPortLinux(path,config);
    #elif defined(PLATFORM_WINDOWS) && !defined(PLATFORM_LINUX)
    pActualPort = new SerialPortWindows(path,config);
    #endif

    try
    {
       if(pActualPort->GetPortState() == PortState::STATE_OPEN)
        {
            this->port = dynamic_cast<SerialPort*>(pActualPort);
            if(this->port){stat = 0;}
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return stat;
}

int SerialDevice::DeletePortInstance(void)
{
    return 0;
}


