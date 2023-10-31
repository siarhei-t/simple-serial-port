/******************************************************************************
* File Name          : SerialPortWindows.hpp
* Author             : 
* Version            : v 1.0
* Description        : header for SerialPortWindows.cpp
*******************************************************************************/
#ifndef SERIAL_PORT_WINDOWS_H
#define SERIAL_PORT_WINDOWS_H

#include <windows.h>
#include "SerialDevice.hpp"

class SerialPortWindows : public SerialPort
{
    public:
    
        SerialPortWindows();
        SerialPortWindows(const std::string& path);
        ~SerialPortWindows();
        
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
        HANDLE port_desc;
        // \brief struct with port configuration
        //struct termios tty;
        // \brief load actual configuration to the tty struct 
        void LoadPortConfiguration();
        // \brief save tty struct as actual configuration
        void SavePortConfiguration();
        // \brief load default configuration to the tty struct 
        void SetDefaultPortConfiguration();
};

#endif /*SERIAL_PORT_WINDOWS_H*/