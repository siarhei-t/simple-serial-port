/**
 * @file serial_port.cpp
 *
 * @brief implementation for class defined in serial_port.hpp
 *
 * @author Siarhei Tatarchanka
 *
 */

#include "../inc/serial_port.hpp"
#include "../inc/sp_error.hpp"
#if defined(__APPLE__) || defined(__linux__)
#include "../inc/platform/sp_posix.hpp"
#elif defined(_WIN32)
#include "../inc/platform/sp_windows.hpp"
#else
#error "target platform not defined."
#endif

using namespace sp;

struct SerialPort::Platform
{
    #if defined(__APPLE__) || defined(__linux__)
    SerialPortPosix port;
    #elif defined(_WIN32)
    SerialPortWindows port;
    #endif
    /// @brief open port
    /// @param path string with path to device
    void openPort(const std::string& path)
    {
        port.openPort(path);
    }
    /// @brief close actual port if opened
    void closePort()
    {
        port.closePort();
    }
    /// @brief setup port with new configuration
    /// @param config struct with port configuration
    void setupPort(const sp::PortConfig& config)
    {
        port.setupPort(config);
    }
    /// @brief write string data to actual port
    /// @param data string object with data to send
    void writeString(const std::string& data)
    {
        port.writeString(data);
    }
    /// @brief write raw data to actual port
    /// @param data string object with data to send
    void writeBinary(const std::vector<std::uint8_t>& data)
    {
        port.writeBinary(data);
    }
    /// @brief read raw data from port
    /// @param data reference to vector with buffer for data
    /// @param length how many bytes we expect to read during timeout
    /// @returns how many bytes we read actually
    size_t readBinary(std::vector<std::uint8_t>& data, size_t length)
    {
        return port.readBinary(data, length);
    }
    /// @brief reset internal OS buffers
    void flushPort()
    {
        port.flushPort();
    }
};

SerialPort::SerialPort(): impl(std::make_unique<Platform>()){}

SerialPort::~SerialPort(){}

SerialPort::SerialPort(std::string name, sp::PortConfig config) : impl(std::make_unique<Platform>())
{
    std::error_code error_code = open(name);
    if(error_code)
    {
        throw std::system_error(error_code);
    }
    error_code = setup(config);
    if(error_code)
    {
        throw std::system_error(error_code);
    }
}

std::error_code SerialPort::open(const std::string name)
{
    std::error_code error_code = std::error_code();
    try
    {
        impl->openPort(name);
        flushPort();
        state = sp::PortState::Open;
        path = name;
    }
    catch (const std::system_error& e)
    {
        error_code = e.code();
    }
    return error_code;
}

void SerialPort::close()
{
    impl->closePort();
    state = sp::PortState::Close;
    path = "NULL";
}

std::error_code SerialPort::setup(sp::PortConfig config)
{
    std::error_code error(0, sp::sp_category());
    try
    {
        impl->setupPort(config);
        this->config = config;
    }
    catch (const std::system_error& e)
    {
        error = e.code();
    }
    return error;
}

void SerialPort::writeString(const std::string& data)
{
    impl->writeString(data);
}

void SerialPort::writeBinary(const std::vector<std::uint8_t>& data)
{
    impl->writeBinary(data);
}

size_t SerialPort::readBinary(std::vector<std::uint8_t>& data, size_t length)
{
    return impl->readBinary(data, length);
}

void SerialPort::flushPort()
{
    impl->flushPort();
}
