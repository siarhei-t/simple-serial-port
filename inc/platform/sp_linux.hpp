/**
 * @file SerialPortLinux.hpp
 *
 * @brief 
 *
 * @author Siarhei Tatarchanka
 *
 */
#ifndef SERIAL_PORT_LINUX_H
#define SERIAL_PORT_LINUX_H

#include <unistd.h>
#include <fcntl.h> 
#include <errno.h>
#include <termios.h>

#include "SerialDevice.hpp"

class SerialPortLinux : public SerialPort
{
    public:
    
        SerialPortLinux():port_desc(-1){};
        ~SerialPortLinux();
        /// @brief open port
        /// @param path string with path to device
        void openPort(const std::string& path);
        /// @brief close actual port if opened
        void closePort();
        /// @brief setup port with new configuration
        /// @param config struct with port configuration
        void setup(const PortConfig& config);
        /// @brief write string data to actual port
        /// @param data string object with data to send
        void writeString(const std::string& data);
        /// @brief write raw data to actual port
        /// @param data string object with data to send
        void writeBinary(const std::vector<uint8_t>& data);
        /// @brief read raw data from port 
        /// @param data reference to vector with buffer for data
        /// @param length how many bytes we expect to read during timeout
        /// @returns how many bytes we read actually
        size_t readBinary(std::vector<uint8_t>& data, size_t length);
    
    private:
        /// @brief opened port file descriptor
        int port_desc;
        /// @brief struct with port configuration
        struct termios tty;
        /// @brief setup parity bits in tty struct
        /// @param parity expected parity mode
        void setParity(const PortParity parity);
        // \brief setup baudrate in tty struct
        // \param baudrate expected port baudrate
        void setBaudRate(const PortBaudRate baudrate);
        // \brief setup data bits in tty struct
        // \param num_of_data_bits expected number of data bits in frame
        void setDataBits(const PortDataBits num_of_data_bits);
        // \brief setup stop bits in tty struct
        // \param num_of_stop_bits expected number of stop bits in frame
        void setStopBits(const PortStopBits num_of_stop_bits);
        // \brief setup timeout in tty struct
        // \param timeout_ms expected timeout i ms for data read
        void setTimeOut(const int timeout_ms);
        // \brief load actual configuration to the tty struct 
        void loadPortConfiguration();
        // \brief save tty struct as actual configuration
        void savePortConfiguration();
        // \brief load default configuration to the tty struct 
        void setDefaultPortConfiguration();
};

#endif /*SERIAL_PORT_LINUX_H*/
