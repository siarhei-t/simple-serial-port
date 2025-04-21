/**
 * @file read_write.hpp
 *
 * @brief 
 *
 * @author Siarhei Tatarchanka
 *
 */

#include <iostream>
#include <string>
#include <system_error>
#include "serial_port.hpp"

int main(int argc, char* argv[])
{
    if(argc != 3)
    {
        std::printf("incorrect arguments list passed, exit...\n");
        return 0;
    }
    // strings with port names, by default it will search in /dev/ on Linux and Apple machines
    std::string port_1_path = argv[1];
    std::string port_2_path  = argv[2];

    // default port config was created in constructor
    sp::PortConfig config;
    std::error_code stat;
    // port instances
    sp::SerialPort test_port_1;
    sp::SerialPort test_port_2;

    stat = test_port_1.open(port_1_path);
    if(stat)
    {
        std::cout<<"failed to open port " + port_1_path + "\n"; 
        std::cout<<"error: "<<stat.message()<<"\n";
        return 0;
    }
    stat = test_port_1.setup(config);
    if(stat)
    {
        std::cout<<"failed to setup port " + port_1_path + "\n"; 
        std::cout<<"error: "<<stat.message()<<"\n";
        return 0;
    }
    stat = test_port_2.open(port_2_path);
    if(stat)
    {
        std::cout<<"failed to open port " + port_2_path + "\n";
        std::cout<<"error: "<<stat.message()<<"\n";
        return 0;
    }
    stat = test_port_2.setup(config);
    if(stat)
    {
        std::cout<<"failed to setup port " + port_2_path + "\n";  
        std::cout<<"error: "<<stat.message()<<"\n";
        return 0;
    }
    if((test_port_1.getState() == sp::PortState::Open) || (test_port_2.getState() == sp::PortState::Open))
    {
        // we will sent data to test_port_1 and read it back in test_port_2
        std::cout<<"port on path " <<test_port_1.getPath()<<" opened successfully." <<"\n";
        std::cout<<"port on path " <<test_port_2.getPath()<<" opened successfully." <<"\n";
        std::string data_to_send = "This is a test string with length more than 32 bytes";
        std::vector<std::uint8_t> data_to_read;
        std::cout<<"DATA SENT :"<<data_to_send<<"\n"<<"\n";
        test_port_1.writeString(data_to_send);
        auto bytes_read = test_port_2.readBinary(data_to_read,data_to_send.size());
        std::cout<<"BYTES READ :"<<bytes_read<<"\n";
        std::string received_data(data_to_read.begin(),data_to_read.end());
        std::cout<<"DATA READ :"<<received_data<<"\n"<<"\n";
        std::cout<<" test finished, exit..." <<"\n";
    }
    test_port_1.close();
    test_port_2.close();
    return 0;
}
