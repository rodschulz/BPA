cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(BPA)

find_package(PCL 1.7 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenMP)

# Add PCL dirs
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add OpenMP flags
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Add NVCC flags
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_20,code=sm_20)

# PCL build type
set(PCL_BUILD_TYPE Release)

# Project's files
file(GLOB BPA_SRC
    "*.h"
    "*.cpp"
    "*.cu"
)

# Define the executable
cuda_add_executable(BPA ${BPA_SRC})
target_link_libraries(BPA ${PCL_LIBRARIES})
