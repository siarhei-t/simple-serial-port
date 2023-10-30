cmake_minimum_required (VERSION 3.15)

project (simple-serial-port)

enable_language(CXX)

set(COMMON_SOURCES
        SerialDevice.cpp
)

if (UNIX)
set(PLATFORM_SOURCES
        SerialPortLinux.cpp
)
endif (UNIX)

add_library (${PROJECT_NAME} STATIC ${COMMON_SOURCES} ${PLATFORM_SOURCES})

if (UNIX)
        target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_LINUX=1)
endif (UNIX)

target_include_directories(${PROJECT_NAME} PRIVATE
        ../inc
        )

