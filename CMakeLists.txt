cmake_minimum_required(VERSION 2.8)
project(ARoboCoreProject NONE)

include(${HFRAMEWORK_PATH}/hFramework.cmake)
enable_module(hCloudClient)
enable_module(hSensors)
enable_module(hModules)
enable_module(hROS)

include_directories(include)
include_directories(src)

file(GLOB all_src
     "include/*.h"
     "src/*.cpp"
)


add_hexecutable(myproject main.cpp ${all_src})
