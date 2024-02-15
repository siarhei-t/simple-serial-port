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


SerialPortLinux::~SerialPortLinux()
{
    closePort();
}

void SerialPortLinux::openPort(const std::string& path)
{
    std::string dev_path = "/dev/" + path;
    port_desc = open(dev_path.c_str(),O_RDWR);   
    if(port_desc < 0)
    {
        throw std::runtime_error(std::string("Could not open device on path :") + path);
    }
    else{state = PortState::Open;}
}

void SerialPortLinux::closePort(void)
{
    if(state != PortState::Close)
    {
        close(port_desc);
        state = PortState::Close;
        port_desc = -1;
    }
}

void SerialPortLinux::setup(const PortConfig &config)
{
    loadPortConfiguration();
    setDefaultPortConfiguration();
    setBaudRate(config.baudrate);
    setDataBits(config.data_bits);
    setParity(  config.parity);
    setStopBits(config.stop_bits);
    setTimeOut( config.timeout_ms);
    savePortConfiguration();
}

void SerialPortLinux::writeString(const std::string& data)
{
    int stat = write(port_desc, data.c_str(), data.size());
    if(stat == -1)
    {
        throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
    }
}

void SerialPortLinux::writeBinary(const std::vector<uint8_t>& data)
{
    int stat = write(port_desc, data.data(), data.size());
    if(stat == -1)
    {
        throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
    }
}

size_t SerialPortLinux::readBinary(std::vector<uint8_t>& data, size_t length)
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

void SerialPortLinux::setParity(const PortParity parity)
{
    switch(parity)
    {
        case PortParity::None:
            this->tty.c_cflag &= ~PARENB;
            break;

        case PortParity::Even:
            this->tty.c_cflag |=  PARENB;
            this->tty.c_cflag &= ~PARODD;
            break;

        case PortParity::Odd:
            this->tty.c_cflag |= PARENB;
            this->tty.c_cflag |= PARODD;
            break;

        default:
            break;
    }
}

void SerialPortLinux::setBaudRate(const PortBaudRate baudrate)
{
    switch(baudrate)
    {
        case PortBaudRate::BD_9600:
            cfsetispeed(&(this->tty), B9600);
            cfsetospeed(&(this->tty), B9600);
            break;
        case PortBaudRate::BD_19200:
            cfsetispeed(&(this->tty), B19200);
            cfsetospeed(&(this->tty), B19200);
            break;
        case PortBaudRate::BD_38400:
            cfsetispeed(&(this->tty), B38400);
            cfsetospeed(&(this->tty), B38400);
            break;
        case PortBaudRate::BD_57600:
            cfsetispeed(&(this->tty), B57600);
            cfsetospeed(&(this->tty), B57600);
            break;
        case PortBaudRate::BD_115200:
            cfsetispeed(&(this->tty), B1152000);
            cfsetospeed(&(this->tty), B1152000);
            break;
        
        default:
            break;
    }
}

void SerialPortLinux::setDataBits(const PortDataBits num_of_data_bits)
{
    this->tty.c_cflag &= ~CSIZE;
    
    switch(num_of_data_bits)
    {
        case PortDataBits::Five:
            this->tty.c_cflag |= CS5;
            break;

        case PortDataBits::Six:
            this->tty.c_cflag |= CS6;
            break;
        case PortDataBits::Seven:
            this->tty.c_cflag |= CS7;
            break;
        case PortDataBits::Eight:
            this->tty.c_cflag |= CS8;
            break;

        default:
            break;    
    }
}

void SerialPortLinux::setStopBits(const PortStopBits num_of_stop_bits)
{
    switch(num_of_stop_bits)
    {
        case PortStopBits::One:
            this->tty.c_cflag &= ~CSTOPB;
            break;

        case PortStopBits::Two:
            this->tty.c_cflag |= CSTOPB;
            break;

        default:
            break;
    }
}

void SerialPortLinux::setTimeOut(const int timeout_ms)
{
    const unsigned char max_timeout = 0xFF;
    this->tty.c_cc[VMIN] = 0;
    int timeout = timeout_ms/100;
    if(timeout > max_timeout)
    {
        throw std::runtime_error(std::string() +"unsupported parameter type passed to :" + __FUNCTION__);
    }
    else
    {
        this->tty.c_cc[VTIME] = timeout;
    }
}

void SerialPortLinux::loadPortConfiguration()
{
    if(this->state == PortState::Open)
    {
        int stat = tcgetattr(this->port_desc, &(this->tty));
        if(stat != 0){throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);}
    }
}

void SerialPortLinux::savePortConfiguration()
{
    if(this->state == PortState::Open)
    {
        int stat = tcsetattr(this->port_desc, TCSANOW, &(this->tty));
        if(stat != 0){throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);}
    }
}

void SerialPortLinux::setDefaultPortConfiguration()
{
    //hardware flow control disabled
    this->tty.c_cflag &= ~CRTSCTS;
    //disable software flow control
    this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // only raw data
    this->tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    // read enabled and ctrl lines ignored  
    this->tty.c_cflag |= CREAD | CLOCAL;
    // all features disabled (echo, new lines, modem modes, etc.)
    this->tty.c_lflag &= ~ICANON;
    this->tty.c_lflag &= ~ECHO;
    this->tty.c_lflag &= ~ECHOE;
    this->tty.c_lflag &= ~ECHONL;
    this->tty.c_lflag &= ~ISIG;
    this->tty.c_oflag &= ~OPOST;
    this->tty.c_oflag &= ~ONLCR;
}
