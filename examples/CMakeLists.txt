cmake_minimum_required (VERSION 3.15)

project (read-write-example)

set(EXECUTABLE ${PROJECT_NAME})

set (DIR_SRCS
        ReadWrite.cpp
    )

add_executable (${EXECUTABLE} ${DIR_SRCS})

add_subdirectory(../src serial-port)

target_link_directories(${EXECUTABLE} PUBLIC src)
target_link_libraries (${EXECUTABLE} simple-serial-port)

#common inclide path
target_include_directories(${EXECUTABLE} PRIVATE
        ../inc    
        )