cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(BPA)

find_package(PCL 1.7 REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(PCL_BUILD_TYPE Release)

file(GLOB BPA_SRC
    "*.h"
    "*.cpp"
    "*.cu"
)

cuda_add_executable(BPA ${BPA_SRC})
target_link_libraries(BPA ${PCL_LIBRARIES})
