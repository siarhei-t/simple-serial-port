cmake_minimum_required (VERSION 3.15)

project (simple-serial-port)

enable_language(CXX)

option(TARGET_WINDOWS "build for Windows" OFF)
option(TARGET_LINUX   "build for Lunux"   OFF)

message("********************************* ")
if(TARGET_WINDOWS)
message("target platform selected as Windows")
set(TARGET_PLATFORM PLATFORM_WINDOWS)
endif()

if(TARGET_LINUX)
message("target platform selected as Linux")
set(TARGET_PLATFORM PLATFORM_LINUX)
set(PLATFORM_SOURCES
        SerialPortLinux.cpp
)
endif()

set(COMMON_SOURCES
        SerialDevice.cpp
)
message("********************************* ")

if(NOT DEFINED TARGET_PLATFORM)
message( SEND_ERROR "target platform is nor defined.")
endif()

add_library (${PROJECT_NAME} STATIC ${COMMON_SOURCES} ${PLATFORM_SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${TARGET_PLATFORM}=1)
target_include_directories(${PROJECT_NAME} PRIVATE
        ../inc
        )

