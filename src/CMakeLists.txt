cmake_minimum_required (VERSION 3.15)

project (simple-serial-port)

enable_language(CXX)

set(PROGRAM_SOURCES
        SerialDevice.cpp
)

add_library (${PROJECT_NAME} STATIC ${PROGRAM_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
        ../inc
        )

