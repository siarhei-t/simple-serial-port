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

SerialPortWindows::SerialPortWindows(const std::string& path, const PortConfig& config)
{
    this->port_desc = INVALID_HANDLE_VALUE;
    this->state     = PortState::STATE_CLOSE;
    try{this->Open(path);}
    catch(const std::exception& e){std::cerr << e.what() << '\n';}
    // set default config in case of success
    if(this->GetPortState() ==  PortState::STATE_OPEN)
    {
        try
        {
            this->Setup(config);
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
    this->Close();
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
    this->LoadPortConfiguration();
    this->SetBaudRate(config.baudrate);
    this->SetDataBits(config.data_bits);
    this->SetParity(  config.parity);
    this->SetStopBits(config.stop_bits);
    this->SavePortConfiguration();
    this->SetTimeOut( config.timeout_ms);
}

void SerialPortWindows::WriteString(const std::string &data)
{
    DWORD bytes_written;
    WINBOOL stat = WriteFile(this->port_desc, data.c_str(),data.size(),&bytes_written,NULL);
    if(stat == 0)
    {
        throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
    }
}

void SerialPortWindows::WriteBinary(const std::vector<uint8_t>& data)
{
    DWORD bytes_written;
    WINBOOL stat = WriteFile(this->port_desc, data.data(),data.size(),&bytes_written,NULL);
    if(stat == 0)
    {
        throw std::runtime_error(std::string() +"error with port access in function :" + __FUNCTION__);
    }
}

size_t SerialPortWindows::Read(std::vector<uint8_t>& data, size_t length)
{
    uint8_t* p_buffer    = new uint8_t[length];
    size_t bytes_to_read = length;
    DWORD bytes_read     = 0;
    
    while(bytes_to_read != 0)
    {
        WINBOOL n = ReadFile(this->port_desc, &p_buffer[bytes_read],length, &bytes_read, NULL);
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

    FlushFileBuffers(this->port_desc);
    return bytes_read;
}

void SerialPortWindows::SetParity(const PortParity parity)
{
    switch(parity)
    {
        case PortParity::P_NONE:
            this->tty.Parity = NOPARITY;
            break;
        case PortParity::P_EVEN:
            this->tty.Parity = EVENPARITY;
            break;
        case PortParity::P_ODD:
            this->tty.Parity = ODDPARITY;
            break;
        default:
            throw std::runtime_error(std::string() +"unsupported parameter type passed to :" + __FUNCTION__);
            break;
    }
}

void SerialPortWindows::SetBaudRate(const PortBaudRate baudrate)
{
    switch(baudrate)
    {
        case PortBaudRate::BD_9600:
            this->tty.BaudRate = CBR_9600;
            break;
        case PortBaudRate::BD_19200:
            this->tty.BaudRate = CBR_19200;
            break;
        case PortBaudRate::BD_38400:
            this->tty.BaudRate = CBR_38400;
            break;
        case PortBaudRate::BD_57600:
            this->tty.BaudRate = CBR_57600;
            break;
        case PortBaudRate::BD_115200:
            this->tty.BaudRate = CBR_115200;
            break;
        default:
            throw std::runtime_error(std::string() +"unsupported parameter type passed to :" + __FUNCTION__);
            break;
    }
}

void SerialPortWindows::SetDataBits(const PortDataBits num_of_data_bits)
{
    switch(num_of_data_bits)
    {
        case PortDataBits::DB_FIVE:
            this->tty.ByteSize = 5;
            break;
        case PortDataBits::DB_SIX:
            this->tty.ByteSize = 6;
            break;
        case PortDataBits::DB_SEVEN:
            this->tty.ByteSize = 7;
            break;
        case PortDataBits::DB_EIGHT:
            this->tty.ByteSize = 8;
            break;
        default:
            throw std::runtime_error(std::string() +"unsupported parameter type passed to :" + __FUNCTION__);
            break;    
    }
}

void SerialPortWindows::SetStopBits(const PortStopBits num_of_stop_bits)
{
    switch(num_of_stop_bits)
    {
        case PortStopBits::SB_ONE:
            this->tty.StopBits = ONESTOPBIT;
            break;
        case PortStopBits::SB_TWO:
            this->tty.StopBits = TWOSTOPBITS;
            break;
        default:
            throw std::runtime_error(std::string() +"unsupported parameter type passed to :" + __FUNCTION__);
            break;
    }
}

void SerialPortWindows::SetTimeOut(const int timeout_ms)
{
    COMMTIMEOUTS timeouts;
    WINBOOL stat = GetCommTimeouts(this->port_desc, &timeouts);
    if(stat == 0)
    {
        throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);
    }
    timeouts.ReadTotalTimeoutConstant    = timeout_ms;
    timeouts.WriteTotalTimeoutConstant   = timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadIntervalTimeout         = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    stat = SetCommTimeouts(this->port_desc, &timeouts);
    if(stat == 0)
    {
        throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);
    }
}

void SerialPortWindows::LoadPortConfiguration()
{   
    if(this->state == PortState::STATE_OPEN)
    {
        WINBOOL stat = GetCommState(this->port_desc, &this->tty);
        if(stat == 0){throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);}
    }
}

void SerialPortWindows::SavePortConfiguration()
{
    if(this->state == PortState::STATE_OPEN)
    {
        WINBOOL stat = SetCommState(this->port_desc, &this->tty);
        if(stat == 0){throw std::runtime_error(std::string() +"internal error in :" + __FUNCTION__);}
    }
}
