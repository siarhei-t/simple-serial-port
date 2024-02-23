/**
 * @file serial_port.hpp
 *
 * @brief 
 *
 * @author Siarhei Tatarchanka
 *
 */

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <cstdint>
#include <string>
#include <vector>

#include "../inc/sp_types.hpp"
#if defined(PLATFORM_LINUX)
#include "../inc/platform/sp_linux.hpp"
#elif defined(PLATFORM_WINDOWS)
#include "../inc/platform/sp_windows.hpp"
#else
#error "target platform not defined."
#endif

class SerialPort
{
    public:
        SerialPort(std::string path, sp::PortConfig config);
        #if defined(PLATFORM_LINUX)
        SerialPortLinux port;
        #elif defined(PLATFORM_WINDOWS)
        SerialPortWindows port;
        #endif

    private:
        std::string path;
};

class SerialDevice
{
    public:
        SerialDevice(){updateAvailableDevices();};
        /// @brief plaform depended call to update list with serial port devices
        void updateAvailableDevices();
        /// @brief request for list with serial port devices in system
        /// @return referense to a vector with devices
        std::vector<std::string>& getListOfAvailableDevices(){return devices;};
        /// @brief request for list with serial port devices in system
        /// @return vector with devices
        std::vector<std::string> getListOfAvailableDevices() const {return devices;};
    
    private:
        /// @brief vertor with actual list of available serial ports
        std::vector<std::string> devices;
};

#endif //SERIAL_PORT_H
