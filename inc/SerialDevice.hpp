/******************************************************************************
* File Name          : SerialDevice.h
* Author             : Siarhei Tatarchanka
* Version            : v 1.0
* Description        : header for SerialDevice.cpp
*******************************************************************************/
#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

#include <cstdint>
#include <string>
#include <vector>

#define RETVAL_SUCCESS 0

enum class PortState
{
    STATE_OPEN,
    STATE_CLOSE
};

enum class PortBaudRate
{
    BD_9600,
    BD_19200,
    BD_38400,
    BD_57600,
    BD_115200
};

enum class PortDataBits
{
    DB_FIVE,
    DB_SIX,
    DB_SEVEN,
    DB_EIGHT,
};

enum class PortParity 
{
    P_NONE,
    P_EVEN,
    P_ODD,
};

enum class PortStopBits
{
    SB_ONE,
    SB_TWO,
};

struct PortConfig
{
    PortBaudRate baudrate;
    PortDataBits data_bits;
    PortParity   parity;
    PortStopBits stop_bits;
};

class SerialPort
{
    public:
        // \brief open port
        // \param string with path to device
        virtual void Open(const std::string& path) = 0;
        // \brief close actual port if opened
        virtual void Close(void) = 0;
        // \brief setup port with new configuration
        // \param struct with new configuration
        virtual void Setup(const PortConfig& config) = 0;
        // \brief write string data to actual port
        // \param data string object with data
        virtual void WriteString(const std::string& data) = 0;
        // \brief write raw data to actual port
        // \param data string object with data
        virtual void WriteBinary(const std::vector<uint8_t>& data) = 0;
        // \brief read data from actual port
        // \param data reference to vector with buffer for data
        // \param length how many bytes we expect to read during timeout
        // \returns how many bytes we read actually
        virtual size_t Read(std::vector<uint8_t>& data, size_t length){return -1;}
        // \brief get actual port state
        // \returns enum class PortState 
        virtual PortState GetPortState() const final
        {
            return this->state;
        }
    
    protected:
        // \brief actual port state
        PortState state = PortState::STATE_CLOSE;
};

class SerialDevice
{
    public:
        SerialDevice();
        ~SerialDevice();

        /// @brief request for all available serial port devices in system
        /// @param devices vector into which ports will be written (if exist)
        void GetListOfAvailableDevices(std::vector<std::string>& devices);
        /// @brief create port instance
        /// @param path string with path to port (can be obtained in GetListOfAvailableDevices)
        /// @return 0 in case of success, other value in case of fault
        int CreatePortInstance(const std::string path);
        /// @brief delete port instance (if exist) 
        /// @return 0 in case of success, other value in case of fault
        int DeletePortInstance(void);
        /// @brief request for pointer to actual serial port instance
        /// @return pointer to port(if exist), nullptr if port does nor exist 
        SerialPort* GetPointerToPort(void); 
    
    private:
        SerialPort* port;
};

#endif /*SERIAL_DEVICE_H*/
