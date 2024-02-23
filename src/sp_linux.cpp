/**
 * @file sp_linux.cpp
 *
 * @brief 
 *
 * @author Siarhei Tatarchanka
 *
 */

#include "../inc/platform/sp_linux.hpp"
#include "../inc/sp_error.hpp"

sp::PortState SerialPortLinux::openPort(const std::string& path)
{
    std::string dev_path = "/dev/" + path;
    port_desc = open(dev_path.c_str(),O_RDWR);   
    if(port_desc < 0){state = sp::PortState::Close;}
    else{state = sp::PortState::Open;}
    return state;
}

void SerialPortLinux::closePort(void)
{
    if(state != sp::PortState::Close)
    {
        close(port_desc);
        state = sp::PortState::Close;
        port_desc = -1;
    }
}

void SerialPortLinux::setup(const sp::PortConfig &config)
{
    loadPortConfiguration();
    setDefaultPortConfiguration();
    setBaudRate(config.baudrate);
    setDataBits(config.data_bits);
    setParity(  config.parity);
    setStopBits(config.stop_bits);
    setTimeOut( config.timeout_ms);
    savePortConfiguration();
}

void SerialPortLinux::writeString(const std::string& data)
{
    if(state == sp::PortState::Open)
    {
        int stat = write(port_desc, data.c_str(), data.size());
        if(stat == -1)
        {
            throw std::system_error(sp::make_error_code(sp::PortErrors::failed_to_write));
        }
    }
}

void SerialPortLinux::writeBinary(const std::vector<uint8_t>& data)
{
    if(state == sp::PortState::Open)
    {
        int stat = write(port_desc, data.data(), data.size());
        if(stat == -1)
        {
            throw std::system_error(sp::make_error_code(sp::PortErrors::failed_to_write));
        }
    }
}

size_t SerialPortLinux::readBinary(std::vector<uint8_t>& data, size_t length)
{
    size_t bytes_to_read = length;
    size_t bytes_read    = 0;
    
    if(state == sp::PortState::Open)
    {
        data.resize(length);
        while(bytes_to_read != 0)
        {
            size_t n = read(this->port_desc, data.data(), bytes_to_read);
            if(n < 0){throw std::system_error(sp::make_error_code(sp::PortErrors::failed_to_read));}
            else if((n > 0) && (n <= bytes_to_read))//reading 
            {
                bytes_to_read = bytes_to_read - n;
                bytes_read    = bytes_read    + n;
            }
            else if(n == 0){break;}//nothing to read
        }
        data.resize(bytes_read);
        tcflush(this->port_desc,TCIOFLUSH);
    }
    return bytes_read;
}

void SerialPortLinux::setParity(const sp::PortParity parity)
{
    switch(parity)
    {
        case sp::PortParity::None:
            this->tty.c_cflag &= ~PARENB;
            break;

        case sp::PortParity::Even:
            this->tty.c_cflag |=  PARENB;
            this->tty.c_cflag &= ~PARODD;
            break;

        case sp::PortParity::Odd:
            this->tty.c_cflag |= PARENB;
            this->tty.c_cflag |= PARODD;
            break;

        default:
            break;
    }
}

void SerialPortLinux::setBaudRate(const sp::PortBaudRate baudrate)
{
    switch(baudrate)
    {
        case sp::PortBaudRate::BD_9600:
            cfsetispeed(&(this->tty), B9600);
            cfsetospeed(&(this->tty), B9600);
            break;
            
        case sp::PortBaudRate::BD_19200:
            cfsetispeed(&(this->tty), B19200);
            cfsetospeed(&(this->tty), B19200);
            break;
            
        case sp::PortBaudRate::BD_38400:
            cfsetispeed(&(this->tty), B38400);
            cfsetospeed(&(this->tty), B38400);
            break;
            
        case sp::PortBaudRate::BD_57600:
            cfsetispeed(&(this->tty), B57600);
            cfsetospeed(&(this->tty), B57600);
            break;
            
        case sp::PortBaudRate::BD_115200:
            cfsetispeed(&(this->tty), B1152000);
            cfsetospeed(&(this->tty), B1152000);
            break;
        
        default:
            break;
    }
}

void SerialPortLinux::setDataBits(const sp::PortDataBits num_of_data_bits)
{
    this->tty.c_cflag &= ~CSIZE;
    
    switch(num_of_data_bits)
    {
        case sp::PortDataBits::Five:
            this->tty.c_cflag |= CS5;
            break;

        case sp::PortDataBits::Six:
            this->tty.c_cflag |= CS6;
            break;
            
        case sp::PortDataBits::Seven:
            this->tty.c_cflag |= CS7;
            break;
            
        case sp::PortDataBits::Eight:
            this->tty.c_cflag |= CS8;
            break;

        default:
            break;    
    }
}

void SerialPortLinux::setStopBits(const sp::PortStopBits num_of_stop_bits)
{
    switch(num_of_stop_bits)
    {
        case sp::PortStopBits::One:
            this->tty.c_cflag &= ~CSTOPB;
            break;

        case sp::PortStopBits::Two:
            this->tty.c_cflag |= CSTOPB;
            break;

        default:
            break;
    }
}

void SerialPortLinux::setTimeOut(const int timeout_ms)
{
    const unsigned char max_timeout = 0xFF;
    this->tty.c_cc[VMIN] = 0;
    int timeout = timeout_ms/100;
    if(timeout > max_timeout)
    {
        throw std::system_error(sp::make_error_code(sp::PortErrors::failed_to_save_cfg));
    }
    else
    {
        this->tty.c_cc[VTIME] = timeout;
    }
}

void SerialPortLinux::loadPortConfiguration()
{
    if(state == sp::PortState::Open)
    {
        int stat = tcgetattr(this->port_desc, &(this->tty));
        if(stat != 0){throw std::system_error(sp::make_error_code(sp::PortErrors::failed_to_load_cfg));}
    }
}

void SerialPortLinux::savePortConfiguration()
{
    if(state == sp::PortState::Open)
    {
        int stat = tcsetattr(this->port_desc, TCSANOW, &(this->tty));
        if(stat != 0){throw std::system_error(sp::make_error_code(sp::PortErrors::failed_to_save_cfg));}
    }
}

void SerialPortLinux::setDefaultPortConfiguration()
{
    //hardware flow control disabled
    this->tty.c_cflag &= ~CRTSCTS;
    //disable software flow control
    this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // only raw data
    this->tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    // read enabled and ctrl lines ignored  
    this->tty.c_cflag |= CREAD | CLOCAL;
    // all features disabled (echo, new lines, modem modes, etc.)
    this->tty.c_lflag &= ~ICANON;
    this->tty.c_lflag &= ~ECHO;
    this->tty.c_lflag &= ~ECHOE;
    this->tty.c_lflag &= ~ECHONL;
    this->tty.c_lflag &= ~ISIG;
    this->tty.c_oflag &= ~OPOST;
    this->tty.c_oflag &= ~ONLCR;
}
