/**
 * @file sp_error.cpp
 *
 * @brief 
 *
 * @author Siarhei Tatarchanka
 *
 */

#include "../inc/sp_error.hpp"

namespace
{
    class sp_category_impl : public std::error_category
    {
        const char* name() const noexcept override { return "SERIAL"; }

        std::string message(int condition) const override
        {
            sp::PortErrors error = static_cast<sp::PortErrors>(condition);
            switch (error)
            {
                case sp::PortErrors::no_error:
                    return "no error";
                    
                case sp::PortErrors::failed_to_open:
                    return "unable to open port with selected path";
                    
                case sp::PortErrors::failed_to_read:
                    return "unable to read from port with selected descriptor";
                    
                case sp::PortErrors::failed_to_write:
                    return "unable to write to port with selected descriptor";
                    
                case sp::PortErrors::failed_to_load_cfg:
                    return "unable to load configuration from port with selected descriptor";
                    
                case sp::PortErrors::failed_to_save_cfg:
                    return "unable to write configuration to port with selected descriptor";
                    
                default:
                    return "unknown error";
            }
        }
    };
}

namespace sp
{
    const std::error_category& sp_category()
    {
        static sp_category_impl obj;
        return obj;
    }
    
    std::error_code make_error_code(PortErrors error) noexcept
    {
        return std::error_code(static_cast<int>(error), sp_category());
    }
}



