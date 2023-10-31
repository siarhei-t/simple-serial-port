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
    this->port_desc = INVALID_HANDLE_VALUE;
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
    this->port_desc = CreateFile(path.c_str(), GENERIC_READ | GENERIC_WRITE,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
    if(this->port_desc == INVALID_HANDLE_VALUE)
    {
        throw std::runtime_error(std::string("Could not open device on path :") + path);
    }
    else
    {
        this->state = PortState::STATE_OPEN;
    }
}

void SerialPortWindows::Close(void)
{
    if(this->port_desc != INVALID_HANDLE_VALUE)
    {
        CloseHandle(this->port_desc);
        this->state = PortState::STATE_CLOSE;
        this->port_desc = INVALID_HANDLE_VALUE;
    }else{return;}
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
    return 0;
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