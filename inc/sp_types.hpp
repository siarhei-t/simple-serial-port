/**
 * @file sp_types.hpp
 *
 * @brief common types definitions for serial port library
 *
 * @author Siarhei Tatarchanka
 *
 */

#ifndef SP_TYPES_H
#define SP_TYPES_H

namespace sp
{
enum class PortState
{
    Open,
    Close
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
    Five,
    Six,
    Seven,
    Eight
};

enum class PortParity
{
    None,
    Even,
    Odd
};

enum class PortStopBits
{
    One,
    Two
};

struct PortConfig
{
    PortBaudRate baudrate = PortBaudRate::BD_9600;
    PortDataBits data_bits = PortDataBits::Eight;
    PortParity parity = PortParity::None;
    PortStopBits stop_bits = PortStopBits::One;
    int timeout_ms = 1000;
};
} // namespace sp

#endif // SP_TYPES_H
