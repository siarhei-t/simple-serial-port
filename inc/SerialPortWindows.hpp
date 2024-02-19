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
    
        SerialPortWindows(){};
        ~SerialPortWindows(){closePort();}
        /// @brief open port
        /// @param path string with path to device
        PortState openPort(const std::string& path) override;
        /// @brief close actual port if opened
        void closePort() override;
        /// @brief setup port with new configuration
        /// @param config struct with port configuration
        void setup(const PortConfig& config) override;
        /// @brief write string data to actual port
        /// @param data string object with data to send
        void writeString(const std::string& data) override;
        /// @brief write raw data to actual port
        /// @param data string object with data to send
        void writeBinary(const std::vector<uint8_t>& data) override;
        /// @brief read raw data from port 
        /// @param data reference to vector with buffer for data
        /// @param length how many bytes we expect to read during timeout
        /// @returns how many bytes we read actually
        size_t readBinary(std::vector<uint8_t>& data, size_t length) override;
    
    private:
        // \brief opened port file descriptor
        HANDLE port_desc = INVALID_HANDLE_VALUE;
        // \brief struct with port configuration
        DCB tty = {};
        /// @brief setup parity bits in tty struct
        /// @param parity expected parity mode
        void setParity(const PortParity parity);
        /// @brief setup baudrate in tty struct
        /// @param baudrate expected port baudrate
        void setBaudRate(const PortBaudRate baudrate);
        /// @brief setup data bits in tty struct
        /// @param num_of_data_bits expected number of data bits in frame
        void setDataBits(const PortDataBits num_of_data_bits);
        /// @brief setup stop bits in tty struct
        /// @param num_of_stop_bits expected number of stop bits in frame
        void setStopBits(const PortStopBits num_of_stop_bits);
        /// @brief setup timeout in tty struct
        /// @param timeout_ms expected timeout i ms for data read
        void setTimeOut(const int timeout_ms);
        /// @brief load actual configuration to the tty struct 
        void loadPortConfiguration();
        /// @brief save tty struct as actual configuration
        void savePortConfiguration();
};

#endif /*SERIAL_PORT_WINDOWS_H*/