cmake_minimum_required(VERSION 3.20)
project(simple-serial-port)

enable_language(CXX)

message("*************************************")
message("Detecting platform...")

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(PLATFORM_NAME "Windows")
    set(PLATFORM_SOURCES src/sp_windows.cpp)
    set(PLATFORM_HEADERS inc/platform/sp_windows.hpp)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PLATFORM_NAME "Linux")
    set(PLATFORM_SOURCES src/sp_posix.cpp)
    set(PLATFORM_HEADERS inc/platform/sp_posix.hpp)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(PLATFORM_NAME "macOS")
    set(PLATFORM_SOURCES src/sp_posix.cpp)
    set(PLATFORM_HEADERS inc/platform/sp_posix.hpp)
else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

message("Target platform: ${PLATFORM_NAME}")
message("*************************************")

set(COMMON_SOURCES
    src/serial_port.cpp
    src/sp_error.cpp
)

set(COMMON_HEADERS
    inc/serial_port.hpp
    inc/sp_types.hpp
    inc/sp_error.hpp
)

add_library(${PROJECT_NAME} STATIC
    ${COMMON_SOURCES}
    ${COMMON_HEADERS}
    ${PLATFORM_SOURCES}
    ${PLATFORM_HEADERS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    inc
    inc/platform
)