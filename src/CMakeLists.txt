SET(execName ZED_Camera_Control)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)


include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
message(state ${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

#FILE(GLOB_RECURSE SRC_FILES src/*.cpp)



ADD_EXECUTABLE(${execName} calibration.cpp)

TARGET_LINK_LIBRARIES(${execName}
        ${ZED_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})

