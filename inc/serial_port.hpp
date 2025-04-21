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
#if defined(PLATFORM_LINUX)
#include "../inc/platform/sp_linux.hpp"
#elif defined(PLATFORM_WINDOWS)
#include "../inc/platform/sp_windows.hpp"
#elif defined(PLATFORM_APPLE)
#include "../inc/platform/sp_apple.hpp"
#else
#error "target platform not defined."
#endif

namespace sp
{
class SerialPort
{
public:
    /// @brief default constructor
    SerialPort() = default;
    /// @brief constructor that will open port
    /// @param name port name to open
    SerialPort(std::string name);
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
#if defined(PLATFORM_LINUX)
    SerialPortLinux port;
#elif defined(PLATFORM_WINDOWS)
    SerialPortWindows port;
#elif defined(PLATFORM_APPLE)
    SerialPortApple port;
#endif

private:
    /// @brief actual port state
    sp::PortState state = sp::PortState::Close;
    /// @brief actual port path
    std::string path = "dev/null";
    /// @brief actual port config
    sp::PortConfig config = sp::PortConfig();
};
} // namespace sp

#endif // SERIAL_PORT_H
