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

using namespace sp;

SerialPort::SerialPort(std::string name)
{
    std::error_code error_code = open(name);
    if(error_code)
    {
        throw std::system_error(error_code);
    }
}

SerialPort::SerialPort(std::string name, sp::PortConfig config)
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
        port.openPort(name);
        port.flushPort();
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
    port.closePort();
    state = sp::PortState::Close;
    path = "dev/null";
}

std::error_code SerialPort::setup(sp::PortConfig config)
{
    std::error_code error(0, sp::sp_category());
    try
    {
        port.setupPort(config);
        this->config = config;
    }
    catch (const std::system_error& e)
    {
        error = e.code();
    }
    return error;
}

