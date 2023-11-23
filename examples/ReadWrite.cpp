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
    SerialPort* port = nullptr;
    SerialDevice device;
    const PortConfig config = 
    {
        .baudrate   = PortBaudRate::BD_9600,
        .data_bits  = PortDataBits::DB_EIGHT,
        .parity     = PortParity::P_NONE,
        .stop_bits  = PortStopBits::SB_ONE,
        .timeout_ms = 1000
    };
    std::vector<std::string> devices;

    device.GetListOfAvailableDevices(devices);

    std::cout<<"List of available ports :"<<std::endl;
    for(auto i = 0; i < devices.size(); i++)
    {
        std::cout<<"dev "<<int(i)<<" : "<<devices[i]<<std::endl;
    }
    if(devices.size() > 0)
    {
        int port_id;
        std::cout<<"please select port to open..."<<std::endl;
        std::cin>>port_id;
        std::cout<<"trying to open port with id "<<int(port_id)<<std::endl;
        if(port_id <= (devices.size() - 1))
        {
            port = device.CreatePortInstance(devices[port_id],config);
        }
    }
    else
    {
        std::cout<<"There is no ports to work with."<<std::endl;
        return 0;
    }
    
    if(port != nullptr)
    {
        std::string testLine = "This is a test string. Abcdefghijklmop";
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
    else
    {
        std::cout<<"Failed to open selected port."<<std::endl;
    }
    
    return 0;
}
