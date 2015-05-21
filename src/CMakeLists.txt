cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(BPA)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB BPA_SRC
    "*.h"
    "*.cpp"
)

add_executable(BPA ${BPA_SRC})
target_link_libraries(BPA ${PCL_LIBRARIES})
