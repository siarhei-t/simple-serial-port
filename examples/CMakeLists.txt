cmake_minimum_required (VERSION 3.15)

project (read-write-example)

set(EXECUTABLE ${PROJECT_NAME})

set (DIR_SRCS
        read_write.cpp
    )

add_executable (${EXECUTABLE} ${DIR_SRCS})

add_subdirectory(../src serial-port)
get_directory_property(TARGET_PLATFORM DIRECTORY ../src DEFINITION TARGET_PLATFORM)

target_link_directories(${EXECUTABLE} PUBLIC src)
target_link_libraries (${EXECUTABLE} simple-serial-port)
target_compile_definitions(${EXECUTABLE} PRIVATE ${TARGET_PLATFORM}=1)

#common inclide path
target_include_directories(${EXECUTABLE} PRIVATE
        ../inc    
        )
