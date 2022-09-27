cmake_minimum_required(VERSION 3.10)
project(fls)

# Compile as c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable GNU extensions so the program remains compliant
# with C++ standards and can also run other systems
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS " -O3 ${SSE_FLAGS} -msse4")

# A special treat for OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(STATUS "\nFound OpenMP! Turning on support for parallelization\n")
endif()

# Specify cmake module path (help to find Glog)
list(APPEND CMAKE_MODULE_PATH "${fls_SOURCE_DIR}/cmake")

# Find required packages
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
if(Sophus_FOUND)
    message("-- Found Sophus")
    get_target_property(Sophus_INCLUDE_DIR Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
    include_directories(AFTER ${Sophus_INCLUDE_DIR})
endif()

# A special treat for OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# A special treat for PCL
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package(PCL 1.10 REQUIRED COMPONENTS
    common
    io
    visualization
    filters
    features
    console
    memory
    keypoints
    registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add FLS library
set(fls_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fls/include)
set(fls_HDR_DIR ${fls_INCLUDE_DIR}/fls)
set(fls_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fls/src)
set(fls_INCLUDES ${fls_INCLUDE_DIR})

set(fls_HDRS
    ${fls_HDR_DIR}/fls.h
    ${fls_HDR_DIR}/fls_utils.h
    fls/include/fls/fls_types.h
    fls/include/fls/fls_common.h
    fls/include/fls/local_parameterization_se3.h)
set(fls_SRCS
    ${fls_SOURCE_DIR}/fls.cpp
    ${fls_SOURCE_DIR}/fls_utils.cpp)

add_library(fls SHARED ${fls_HDRS} ${fls_SRCS})
target_include_directories(fls
    PUBLIC ${fls_INCLUDES})
target_link_libraries(fls
    Eigen3::Eigen
    ceres
    Boost::boost
    Boost::thread
    glog gflags
    ${PCL_LIBRARIES}
    Sophus::Sophus)

# Additional compilation flags to enable OpenMP support
if(OPENMP_FOUND)
    message("Found OpenMP!")
    set_target_properties(fls PROPERTIES
        COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
    target_link_libraries(fls ${OpenMP_CXX_FLAGS})
endif()

# Install FLS library
install(TARGETS fls DESTINATION /usr/local/lib)
install(FILES ${fls_HDRS} DESTINATION /usr/local/include/fls)

# Build example directory (finish later)
option(BUILD_BENCHMARK "Build benchmark tools" OFF)
if (BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()
