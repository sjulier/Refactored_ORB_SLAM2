cmake_minimum_required(VERSION 3.1)

project(ORB_SLAM2)

message("Build type: " ${CMAKE_BUILD_TYPE})

# Set the language support level
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# Set where to install stuff
set(BUILD_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../Install)
list(APPEND CMAKE_PREFIX_PATH "${BUILD_INSTALL_PREFIX}")

# Set where the get the prebuilt third party libraries from
set(THIRD_PARTY_BUILT_LIBRARY_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../Develop/${CMAKE_BUILD_TYPE})
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.15.0")
   list(PREPEND CMAKE_SYSTEM_PREFIX_PATH "${THIRD_PARTY_BUILT_LIBRARY_PREFIX}")
else()
   set(CMAKE_SYSTEM_PREFIX_PATH "${THIRD_PARTY_BUILT_LIBRARY_PREFIX};${CMAKE_SYSTEM_PREFIX_PATH}")
endif()
message(CMAKE_SYSTEM_PREFIX_PATH=${CMAKE_SYSTEM_PREFIX_PATH})

# Default installation and search libraries where the yaml files will be stored
set(DEFAULT_ORB_SETTINGS_DIR "${BUILD_INSTALL_PREFIX}/etc/orbslam2")

# Set the vocabulary
set(DEFAULT_ORB_VOCABULARY_DIR "${BUILD_INSTALL_PREFIX}/var/lib/orbslam2")
string(CONCAT DEFAULT_ORB_VOCABULARY ${DEFAULT_ORB_VOCABULARY_DIR} "/ORBvoc.txt")
string(CONCAT DEFAULT_BINARY_ORB_VOCABULARY ${DEFAULT_ORB_VOCABULARY_DIR} "/ORBvoc.bin")
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
   add_compile_definitions(DEFAULT_ORB_VOCABULARY="${DEFAULT_ORB_VOCABULARY}")
   add_compile_definitions(DEFAULT_BINARY_ORB_VOCABULARY="${DEFAULT_BINARY_ORB_VOCABULARY}")
else()
   add_definitions(-DDEFAULT_ORB_VOCABULARY="${DEFAULT_ORB_VOCABULARY}")
   add_definitions(-DDEFAULT_BINARY_ORB_VOCABULARY="${DEFAULT_BINARY_ORB_VOCABULARY}")
endif()

# postfix, based on type
set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
set(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
if(CMAKE_BUILD_TYPE MATCHES Release)
  set(EXE_POSTFIX)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  set(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  set(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
elseif(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  set(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
endif(CMAKE_BUILD_TYPE MATCHES Release)

# Sort out the OpenGL warning
if (POLICY CMP0072)
   set(OpenGL_GL_PREFERENCE LEGACY)
endif()

# Find OpenCV 4.0; the fallback is OpenCV 3.0
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_package(OpenCV 3.0 HINTS "/usr/local/opt/opencv@3" REQUIRED)
  else()
    find_package(OpenCV 3.0 REQUIRED)
  endif()
endif()
message(STATUS "OPENCV VERSION:" ${OpenCV_VERSION})

# Find the other necessary packages
find_package(Eigen3 REQUIRED)

# Backwards compatibilty for eigen
if (TARGET Eigen3::Eigen)
  set(EIGEN3_EIGEN_LIBS Eigen3::Eigen)
else()
  set(EIGEN3_EIGEN_LIBS )
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

find_package(Boost REQUIRED COMPONENTS filesystem)

# Find the required packages; these are from ThirdParty
find_package(Pangolin REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(DLib REQUIRED)
find_package(g2o REQUIRED)

# Add the libraries and examples
add_subdirectory(Libraries)
add_subdirectory(Examples)
add_subdirectory(Vocabulary)

# Third party libraries are built separately to speed things up
