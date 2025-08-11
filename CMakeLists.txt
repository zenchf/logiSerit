cmake_minimum_required(VERSION 3.5) # Use a more recent version of CMake
project(eski_kodlar)

# Set CMake policies
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 NEW) # Modern CUDA handling
endif()
if(POLICY CMP0153)
    cmake_policy(SET CMP0153 NEW) # Use execute_process() instead of exec_program
endif()
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW) # Modern Boost handling
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

# Set CUDA paths explicitly
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.1")
set(CMAKE_PREFIX_PATH "/usr/local/cuda-12.1" ${CMAKE_PREFIX_PATH})

# Enable CUDA as a language
enable_language(CUDA)

# Find required packages
find_package(ZED 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    /home/legaca/async-tcp/async-sockets
)

# Add executables
add_executable(engel engel.cpp)
add_executable(dav dav.cpp)
add_executable(serit serit720.cpp)
add_executable(serit_sol serit720sol.cpp)
add_executable(zedcam zedcam.cpp)

# Link libraries to all targets
foreach(target engel dav serit zedcam serit_sol)
    target_link_libraries(${target}
        ${ZED_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_CUDART_LIBRARY}
        ${OpenCV_LIBS}
    )
endforeach()
