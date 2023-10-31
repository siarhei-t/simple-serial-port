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
    SerialPort* port;
    SerialDevice device;
    std::vector<std::string> devices;

    device.GetListOfAvailableDevices(devices);
    
    std::cout<<"*************************"<<std::endl;
    std::cout<<"List of available ports :"<<std::endl;
    for(auto i = 0; i < devices.size();i++)
    {
        std::cout<<"dev "<<int(i)<<" : "<<devices[i]<<std::endl;
    }
    std::cout<<"*************************"<<std::endl;
    
    /*
    if(devices.size() > 0)
    {
        std::cout<<"opening first available port..."<<std::endl;
        device.CreatePortInstance(devices[0]);
        port = device.GetPointerToPort();
    }
    else
    {
        std::cout<<"There is no ports to work with."<<std::endl;
        return 0;
    }

    if(port != nullptr)
    {
        std::string testLine = "This is a test string.";
        std::vector<uint8_t> buffer;

        std::cout<<"opened port with name : " <<devices[0]<<std::endl;
        std::cout<<"sent: "<<testLine<<std::endl;
        port->WriteString(testLine);
        size_t bytes_read =  port->Read(buffer,testLine.length());
        std::cout<<"bytes read : "<<int(bytes_read)<<std::endl;
        
        if(bytes_read > 0)
        {
            std::string receivedData(buffer.begin(),buffer.end());
            std::cout<<"received: "<<receivedData<<std::endl;
        }
        else
        {
            std::cout<<"nothing was read."<<std::endl;
        }
        
        port->Close();
        
    }
    */
    return 0;
}
