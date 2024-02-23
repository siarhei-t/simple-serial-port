/**
 * @file sp_error.hpp
 *
 * @brief 
 *
 * @author Siarhei Tatarchanka
 *
 */

#ifndef SP_ERROR_H
#define SP_ERROR_H

#include <system_error>

namespace sp
{
    enum class PortErrors
    {
        no_error,
        failed_to_open,
        failed_to_read,
        failed_to_write,
        failed_to_load_cfg,
        failed_to_save_cfg
   };
}

#endif //SP_ERROR_H
