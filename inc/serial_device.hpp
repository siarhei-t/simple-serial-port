/**
 * @file serial_device.hpp
 *
 * @brief serial device interface class definition
 *
 * @author Siarhei Tatarchanka
 *
 */

#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

#include <string>
#include <vector>

namespace sp
{
class SerialDevice
{
public:
    SerialDevice() { updateAvailableDevices(); };
    /// @brief platform depended call to update list with serial port devices
    void updateAvailableDevices();
    /// @brief request for list with serial port devices in system
    /// @return reference to a vector with devices
    std::vector<std::string>& getListOfAvailableDevices() { return devices; };
    /// @brief request for list with serial port devices in system
    /// @return vector with devices
    std::vector<std::string> getListOfAvailableDevices() const
    {
        return devices;
    };

private:
    /// @brief vector with actual list of available serial ports
    std::vector<std::string> devices;
};
} // namespace sp

#endif // SERIAL_PORT_H
