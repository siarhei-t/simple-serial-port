/**
 * @file sp_error.cpp
 *
 * @brief custom system error implementation for serial port library
 *
 * @author Siarhei Tatarchanka
 *
 */

#include "../inc/sp_error.hpp"

namespace
{
class sp_category_impl : public std::error_category
{
    #if defined(__APPLE__) || defined(__linux__)
    const char* name() const noexcept override { return "posix serial"; }
    #elif defined(_WIN32)
    const char* name() const noexcept override { return "windows serial"; }
    #else
    #error "target platform not defined."
    #endif
    std::string message(int condition) const override
    {
        return std::system_category().message(condition);
    }
};
} // namespace

namespace sp
{
const std::error_category& sp_category()
{
    static sp_category_impl obj;
    return obj;
}
} // namespace sp
