/******************************************************************************
* File Name          : SerialPort.cpp
* Author             :
* Version            : v 1.0
* Description        :
*******************************************************************************/

#include <iostream>
#include <system_error>
#include <stdexcept>

#include "SerialPortWindows.hpp"

SerialPortWindows::SerialPortWindows()
{
    this->port_desc = -1;
    this->state     = PortState::STATE_CLOSE;
}

SerialPortWindows::SerialPortWindows(const std::string& path)
{
    try
    {
        this->Open(path);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    // set default config in case of success
    if(this->GetPortState() ==  PortState::STATE_OPEN)
    {
        try
        {
            this->LoadPortConfiguration();
            this->SetDefaultPortConfiguration();
            this->SavePortConfiguration();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            this->Close();
        }
    }
}

SerialPortWindows::~SerialPortWindows()
{

}

void SerialPortWindows::Open(const std::string& path)
{

}

void SerialPortWindows::Close(void)
{

}

void SerialPortWindows::Setup(const PortConfig &config)
{

}

void SerialPortWindows::WriteString(const std::string &data)
{

}

void SerialPortWindows::WriteBinary(const std::vector<uint8_t>& data)
{

}

size_t SerialPortWindows::Read(std::vector<uint8_t>& data, size_t length)
{

}

void SerialPortWindows::LoadPortConfiguration()
{   

}

void SerialPortWindows::SavePortConfiguration()
{

}

void SerialPortWindows::SetDefaultPortConfiguration()
{
    
}