/******************************************************************************
* File Name          : SerialDevice.cpp
* Author             : Siarhei Tatarchanka
* Version            : v 1.0
* Description        : 
*******************************************************************************/

#include <iostream>
#include "SerialDevice.hpp"
#include <iostream>
#if defined(PLATFORM_LINUX) && !defined(PLATFORM_WINDOWS)
#include <sys/types.h>
#include <dirent.h>
#include "SerialPortLinux.hpp"
static std::vector<SerialPortLinux*> platform_ports;
#elif defined(PLATFORM_WINDOWS) && !defined(PLATFORM_LINUX)
#include "SerialPortWindows.hpp"
static std::vector<SerialPortWindows*> platform_ports;
#else
static std::vector<SerialPort*> platform_ports;
#endif

SerialDevice::SerialDevice()
{
    //nothing to create
}

SerialDevice::~SerialDevice()
{
}

void SerialDevice::GetListOfAvailableDevices(std::vector<std::string> &devices)
{
    devices.clear();

    #if defined(PLATFORM_WINDOWS) && !defined(PLATFORM_LINUX)
    wchar_t* dev_path = new wchar_t[256];

    for (auto i = 0; i < 255; i++)
    {
        std::string device = "COM" + std::to_string(i);
        std::wstring converted_device =  std::wstring(device.begin(),device.end());
        DWORD result = QueryDosDevice(converted_device.c_str(),dev_path, 256);
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

void SerialDevice::GetListOfCreatedDevices(std::vector<std::string> &devices)
{
    devices.clear();
    for(auto i = 0; i < this->devices.size(); i++)
    {
        devices.push_back(this->devices[i]);
    }
}

SerialPort* SerialDevice::CreatePortInstance(const std::string path,const PortConfig& config)
{
    SerialPort* created_port = nullptr;
    
    return created_port;
}

