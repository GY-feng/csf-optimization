cmake_minimum_required(VERSION 3.10)
project(CSF LANGUAGES CXX)

option(CSF_ENABLE_CUDA "Build CUDA acceleration kernels" OFF)
option(BUILD_DEMO "Build csfdemo executable" OFF)
set(BUILD_SHARED_LIBS "Build as shared library" OFF)

if(CSF_ENABLE_CUDA)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    add_compile_definitions("CSF_ENABLE_CUDA")
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    add_compile_definitions("CSF_USE_OPENMP")
endif()

add_subdirectory(src)

if(BUILD_DEMO)
    add_subdirectory(CSFDemo)
endif()
