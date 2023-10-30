/******************************************************************************
* File Name          : SerialPort.cpp
* Author             :
* Version            : v 1.0
* Description        :
*******************************************************************************/

#include <iostream>
#include <system_error>
#include <stdexcept>

#include "SerialPortLinux.hpp"

SerialPortLinux::SerialPortLinux()
{
    this->port_desc = -1;
    this->state     = PortState::STATE_CLOSE;
}

SerialPortLinux::SerialPortLinux(const std::string& path)
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

SerialPortLinux::~SerialPortLinux()
{

}

void SerialPortLinux::Open(const std::string& path)
{
    std::string dev_path = "/dev/"+ path;
    this->port_desc = open(dev_path.c_str(),O_RDWR);   
    if(this->port_desc < 0)
    {
        //we have errors
        throw std::runtime_error(std::string("Could not open device on path :") + path);
    }
    else
    {
        this->state = PortState::STATE_OPEN;
    }
}

void SerialPortLinux::Close(void)
{
    if(this->port_desc != -1)
    {
        close(this->port_desc);
        this->state = PortState::STATE_CLOSE;
        this->port_desc = -1;
    }else{return;}
}

void SerialPortLinux::Setup(const PortConfig &config)
{

}

void SerialPortLinux::WriteString(const std::string &data)
{
    int stat = write(this->port_desc, data.c_str(), data.size());
    if(stat == -1)
    {
        throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
    }
}

void SerialPortLinux::WriteBinary(const std::vector<uint8_t>& data)
{
    int stat = write(this->port_desc, data.data(), data.size());
    if(stat == -1)
    {
        throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
    }
}

size_t SerialPortLinux::Read(std::vector<uint8_t>& data, size_t length)
{
    uint8_t* p_buffer    = new uint8_t[length];
    size_t bytes_to_read = length;
    size_t bytes_read    = 0;
    
    while(bytes_to_read != 0)
    {
        size_t n = read(this->port_desc, &p_buffer[bytes_read], bytes_to_read);
        if(n < 0) // error with port access
        {
            delete[] p_buffer;
            throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
        }
        else if((n > 0) && (n <= bytes_to_read)) //reading 
        {
            bytes_to_read = bytes_to_read - n;
            bytes_read    = bytes_read    + n;
        }
        else if(n == 0) //nothing to read
        {
            break;
        }  
    }    
    if (bytes_read > 0)
    {
        data.insert(data.begin(),&p_buffer[0],p_buffer + bytes_read);
    }
    delete[] p_buffer;

    tcflush(this->port_desc,TCIOFLUSH);
    return bytes_read;
}

void SerialPortLinux::LoadPortConfiguration()
{   
    if(this->state == PortState::STATE_OPEN)
    {
        int stat = tcgetattr(this->port_desc, &(this->tty));
        if(stat != 0)
        {
            throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);
        }
    }
}

void SerialPortLinux::SavePortConfiguration()
{
    if(this->state == PortState::STATE_OPEN)
    {
        int stat = tcsetattr(this->port_desc, TCSANOW, &(this->tty));
        if(stat != 0)
        {
            throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);
        }
    }
}

void SerialPortLinux::SetDefaultPortConfiguration()
{
    
    // default config :
    // 1 start bit, 1 stop bit,8 data bits, no parity bits
    // baudrate 19200
    
    this->tty.c_cflag &= ~PARENB;
    this->tty.c_cflag &= ~CSTOPB;
    this->tty.c_cflag &= ~CSIZE;
    this->tty.c_cflag |= CS8;
    this->tty.c_cflag &= ~CRTSCTS;
    this->tty.c_cflag |= CREAD | CLOCAL;
    this->tty.c_lflag &= ~ICANON;
    this->tty.c_lflag &= ~ECHO;
    this->tty.c_lflag &= ~ECHOE;
    this->tty.c_lflag &= ~ECHONL;
    this->tty.c_lflag &= ~ISIG;
    this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    this->tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    this->tty.c_oflag &= ~OPOST;
    this->tty.c_oflag &= ~ONLCR;
    this->tty.c_cc[VTIME] = 20;
    this->tty.c_cc[VMIN] = 0;
    cfsetispeed(&(this->tty), B19200);
    cfsetospeed(&(this->tty), B19200);
}