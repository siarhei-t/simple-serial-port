cmake_minimum_required (VERSION 3.20)

project (read-write-example)
add_executable (${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
        read_write.cpp
)

add_subdirectory(../ serial-port)
target_link_libraries (${PROJECT_NAME} simple-serial-port)

#common include path
target_include_directories(${PROJECT_NAME} PRIVATE
        ../inc
        )
