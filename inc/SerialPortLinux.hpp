/******************************************************************************
* File Name          : SerialPortLinux.hpp
* Author             : 
* Version            : v 1.0
* Description        : header for SerialPortLinux.cpp
*******************************************************************************/
#ifndef SERIAL_PORT_H_INCLUDED
#define SERIAL_PORT_H_INCLUDED

#include <unistd.h>
#include <fcntl.h> 
#include <errno.h>
#include <termios.h>

#include "SerialDevice.hpp"

class SerialPortLinux : public SerialPort
{
    public:
    
        SerialPortLinux();
        SerialPortLinux(const std::string& path);
        ~SerialPortLinux();
        
        // \brief open port
        // \param string with path to device
        void Open(const std::string& path);
        // \brief close actual port if opened
        void Close(void);
        // \brief setup port with new configuration
        // \param struct with new configuration
        void Setup(const PortConfig& config);
        // \brief write string data to actual port
        // \param data string object with data
        void WriteString(const std::string& data);
        // \brief write raw data to actual port
        // \param data string object with data
        void WriteBinary(const std::vector<uint8_t>& data);
        // \brief read data from actual port
        // \param data reference to vector with buffer for data
        // \param length how many bytes we expect to read during timeout
        // \returns how many bytes we read actually
        size_t Read(std::vector<uint8_t>& data, size_t length);
    
    private:
        // \brief opened port file descriptor
        int port_desc;
        // \brief struct with port configuration
        struct termios tty;
        // \brief load actual configuration to the tty struct 
        void LoadPortConfiguration();
        // \brief save tty struct as actual configuration
        void SavePortConfiguration();
        // \brief load default configuration to the tty struct 
        void SetDefaultPortConfiguration();
};

#endif /*SERIAL_PORT_H_INCLUDED*/