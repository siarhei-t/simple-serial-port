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

PortState SerialPortWindows::openPort(const std::string& path)
{
    std::wstring converted_path =  std::wstring(path.begin(),path.end());
    LPCWSTR file_name = converted_path.c_str();
    port_desc = CreateFile(path.c_str(), GENERIC_READ | GENERIC_WRITE,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
    if(this->port_desc == INVALID_HANDLE_VALUE){state = PortState::Close;}
    else{state = PortState::Open;}
    
    return state;
}

void SerialPortWindows::closePort(void)
{
    if(state != PortState::Close)
    {
        CloseHandle(port_desc);
        state = PortState::Close;
        port_desc = INVALID_HANDLE_VALUE;
    }
}

void SerialPortWindows::setup(const PortConfig &config)
{
    loadPortConfiguration();
    setBaudRate(config.baudrate);
    setDataBits(config.data_bits);
    setParity(  config.parity);
    setStopBits(config.stop_bits);
    savePortConfiguration();
    setTimeOut( config.timeout_ms);
}

void SerialPortWindows::writeString(const std::string &data)
{
    if(state == PortState::Open)
    {
        DWORD bytes_written;
        WINBOOL stat = WriteFile(port_desc, data.c_str(),data.size(),&bytes_written,NULL);
        if(stat == 0)
        {
            throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
        }
    }
}

void SerialPortWindows::writeBinary(const std::vector<uint8_t>& data)
{
    if(state == PortState::Open)
    {
        DWORD bytes_written;
        WINBOOL stat = WriteFile(port_desc, data.data(),data.size(),&bytes_written,NULL);
        if(stat == 0)
        {
            throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
        }   
    }
}

size_t SerialPortWindows::readBinary(std::vector<uint8_t>& data, size_t length)
{
    uint8_t* p_buffer    = new uint8_t[length];
    size_t bytes_to_read = length;
    DWORD bytes_read     = 0;
    
    while(bytes_to_read != 0)
    {
        WINBOOL n = ReadFile(port_desc, &p_buffer[bytes_read],length, &bytes_read, NULL);
        if(n == 0) // error with port access
        {
            delete[] p_buffer;
            throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
        }
        else if((bytes_read > 0) && (bytes_read <= bytes_to_read)) //reading 
        {
            bytes_to_read = bytes_to_read - bytes_read;
        }
        else if(bytes_read == 0) //nothing to read
        {
            break;
        }  
    }    
    if (bytes_read > 0)
    {
        data.insert(data.begin(),&p_buffer[0],p_buffer + bytes_read);
    }
    delete[] p_buffer;

    FlushFileBuffers(port_desc);
    return bytes_read;
}

void SerialPortWindows::setParity(const PortParity parity)
{
    switch(parity)
    {
        case PortParity::None:
            tty.Parity = NOPARITY;
            break;
        
        case PortParity::Even:
            tty.Parity = EVENPARITY;
            break;
        
        case PortParity::Odd:
            tty.Parity = ODDPARITY;
            break;
        
        default:
            break;
    }
}

void SerialPortWindows::setBaudRate(const PortBaudRate baudrate)
{
    switch(baudrate)
    {
        case PortBaudRate::BD_9600:
            tty.BaudRate = CBR_9600;
            break;
        
        case PortBaudRate::BD_19200:
            tty.BaudRate = CBR_19200;
            break;

        case PortBaudRate::BD_38400:
            tty.BaudRate = CBR_38400;
            break;
        
        case PortBaudRate::BD_57600:
            tty.BaudRate = CBR_57600;
            break;
        
        case PortBaudRate::BD_115200:
            tty.BaudRate = CBR_115200;
            break;
        
        default:
            break;
    }
}

void SerialPortWindows::setDataBits(const PortDataBits num_of_data_bits)
{
    switch(num_of_data_bits)
    {
        case PortDataBits::Five:
            tty.ByteSize = 5;
            break;

        case PortDataBits::Six:
            tty.ByteSize = 6;
            break;

        case PortDataBits::Seven:
            tty.ByteSize = 7;
            break;
        
        case PortDataBits::Eight:
            tty.ByteSize = 8;
            break;
        
        default:
            break;    
    }
}

void SerialPortWindows::setStopBits(const PortStopBits num_of_stop_bits)
{
    switch(num_of_stop_bits)
    {
        case PortStopBits::One:
            tty.StopBits = ONESTOPBIT;
            break;

        case PortStopBits::Two:
            tty.StopBits = TWOSTOPBITS;
            break;

        default:
            break;
    }
}

void SerialPortWindows::setTimeOut(const int timeout_ms)
{
    COMMTIMEOUTS timeouts;
    WINBOOL stat = GetCommTimeouts(port_desc, &timeouts);
    if(stat == 0)
    {
        throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);
    }
    timeouts.ReadTotalTimeoutConstant    = timeout_ms;
    timeouts.WriteTotalTimeoutConstant   = timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadIntervalTimeout         = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    stat = SetCommTimeouts(port_desc, &timeouts);
    if(stat == 0)
    {
        throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);
    }
}

void SerialPortWindows::loadPortConfiguration()
{   
    if(state == PortState::Open)
    {
        WINBOOL stat = GetCommState(port_desc, &tty);
        if(stat == 0){throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);}
    }
}

void SerialPortWindows::savePortConfiguration()
{
    if(this->state == PortState::Open)
    {
        WINBOOL stat = SetCommState(port_desc, &tty);
        if(stat == 0){throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);}
    }
}
