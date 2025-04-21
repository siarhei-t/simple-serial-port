/**
 * @file serial_port.hpp
 *
 * @brief serial port interface class definition
 *
 * @author Siarhei Tatarchanka
 *
 */

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <vector>
#include <system_error>
#include "../inc/sp_types.hpp"

namespace sp
{
class SerialPort
{
public:
    /// @brief default constructor
    SerialPort();
    /// @brief constructor that will open port and configure it
    /// @param name port name to open
    /// @param config port configuration
    SerialPort(std::string name, sp::PortConfig config);
    /// @brief open port with passed name
    /// @param name port name
    /// @return error code
    std::error_code open(const std::string name);
    /// @brief close port
    void close();
    /// @brief setup port with passed configuration (if port is open)
    /// @param config port configuration
    /// @return error enum
    std::error_code setup(sp::PortConfig config);
    /// @brief request for port system path
    /// @return actual path
    std::string getPath() const { return path; };
    /// @brief request for port configuration
    /// @return struct with configuration
    sp::PortConfig getConfig() const { return config; };
    /// @brief request for port state
    /// @return actual port state
    sp::PortState getState() const { return state; };
    /// @brief write string data to actual port
    /// @param data string object with data to send
    void writeString(const std::string& data);
    /// @brief write raw data to actual port
    /// @param data string object with data to send
    void writeBinary(const std::vector<std::uint8_t>& data);
    /// @brief read raw data from port
    /// @param data reference to vector with buffer for data
    /// @param length how many bytes we expect to read during timeout
    /// @returns how many bytes we read actually
    size_t readBinary(std::vector<std::uint8_t>& data, size_t length);
    /// @brief reset internal OS buffers
    void flushPort();

private:
    /// @brief class for platform depended serial port implementation
    struct Platform;
    /// @brief unique pointer to platform depended serial port implementation
    std::unique_ptr<Platform> impl;  
    /// @brief actual port state
    sp::PortState state = sp::PortState::Close;
    /// @brief actual port path
    std::string path = "NULL";
    /// @brief actual port config
    sp::PortConfig config = sp::PortConfig();
};
} // namespace sp

#endif // SERIAL_PORT_H
