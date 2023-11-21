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
        SerialPortWindows(const std::string& path, const PortConfig& config);
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
        DCB tty;
         // \brief setup parity bits in tty struct
        // \param parity expected parity mode
        void SetParity(const PortParity parity);
        // \brief setup baudrate in tty struct
        // \param baudrate expected port baudrate
        void SetBaudRate(const PortBaudRate baudrate);
        // \brief setup data bits in tty struct
        // \param num_of_data_bits expected number of data bits in frame
        void SetDataBits(const PortDataBits num_of_data_bits);
        // \brief setup stop bits in tty struct
        // \param num_of_stop_bits expected number of stop bits in frame
        void SetStopBits(const PortStopBits num_of_stop_bits);
        // \brief setup timeout in tty struct
        // \param timeout_ms expected timeout i ms for data read
        void SetTimeOut(const int timeout_ms);
        // \brief load actual configuration to the tty struct 
        void LoadPortConfiguration();
        // \brief save tty struct as actual configuration
        void SavePortConfiguration();
};

#endif /*SERIAL_PORT_WINDOWS_H*/