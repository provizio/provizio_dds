cmake_minimum_required(VERSION 3.15)
project(PackageTest CXX)

find_package(provizio_dds CONFIG REQUIRED)

add_executable(example src/example.cpp)
target_link_libraries(example provizio_dds::provizio_dds 
                              provizio_dds::provizio_dds_types 
                              provizio_dds::fastrtps 
                              provizio_dds::fastcdr
)
