cmake_minimum_required(VERSION 3.1)

project(ORB_SLAM2)

message("Build type: " ${CMAKE_BUILD_TYPE})

set(BUILD_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../Develop/${CMAKE_BUILD_TYPE})
list(APPEND CMAKE_PREFIX_PATH "${BUILD_INSTALL_PREFIX}")

# Set the language support level
set(CMAKE_C_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)

# Find OpenCV 4.0; the fallback is OpenCV 3.0
find_package(OpenCV 4.0)
if(NOT OpenCV_FOUND)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_package(OpenCV 3.0 HINTS "/usr/local/opt/opencv@3" REQUIRED)
  else()
    find_package(OpenCV 3.0 REQUIRED)
  endif()
endif()
message(STATUS "OPENCV VERSION:" ${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(DLib REQUIRED)
find_package(g2o REQUIRED)

add_subdirectory(Libraries)
add_subdirectory(Examples)
