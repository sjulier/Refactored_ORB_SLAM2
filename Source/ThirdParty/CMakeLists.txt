cmake_minimum_required(VERSION 3.20)

project(ThirdParty)

# Set the language support level
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# Set where intermediate builds end up
cmake_path(SET BUILD_INSTALL_PREFIX NORMALIZE ${ThirdParty_SOURCE_DIR}/../../Develop/${CMAKE_BUILD_TYPE})

# Build shared libraries or not?
if(NOT DEFINED BUILD_SHARED_LIBRARIES)
  set(BUILD_SHARED_LIBRARIES NO)
endif()

# Set common cmake arguments for all external projects
set(common_cmake_args
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DCMAKE_INSTALL_PREFIX=${BUILD_INSTALL_PREFIX}
  -DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBRARIES}
  )

# Set the toolchain file if it's defined
if (DEFINED CMAKE_TOOLCHAIN_FILE)
  list(PREPEND common_cmake_args
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
  )
endif()

# Set VCKPG installation directory on Windows
if(WIN32)
  cmake_path(SET NORMALIZED_VCPKG_INSTALLED_DIR NORMALIZE ${VCPKG_INSTALLED_DIR})
  list(PREPEND common_cmake_args
    -DVCPKG_INSTALLED_DIR=${NORMALIZED_VCPKG_INSTALLED_DIR}
  )
endif()

# Set up external project support
include(ExternalProject)

# g2o
set(g2o_cmake_args ${common_cmake_args})
list(APPEND g2o_cmake_args
  -DBUILD_LGPL_SHARED_LIBS=${BUILD_SHARED_LIBRARIES}
  -DBUILD_WITH_MARCH_NATIVE=NO
  -DG2O_BUILD_APPS=NO
  -DG2O_BUILD_EXAMPLES=NO
  -DG2O_BUILD_SCLAM2D_TYPES=NO
  -DG2O_BUILD_SLAM2D_TYPES=NO
  -DG2O_USE_LOGGING=NO
  -DTEST_FLAG=ON)

ExternalProject_Add(g2o
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/g2o/g2o-20241228_git  
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/g2o/g2o-20241228_git  
  INSTALL_DIR ${BUILD_INSTALL_PREFIX}
  CMAKE_ARGS ${g2o_cmake_args}
)

# DLib
set(dlib_cmake_args ${common_cmake_args})

list(APPEND dlib_cmake_args
  -DBUILD_DUtilsCV=YES
  -DBUILD_DVision=YES
)
  
ExternalProject_Add(DLib
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/DLib/DLib-local
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/DLib/DLib-local
  INSTALL_DIR ${BUILD_INSTALL_PREFIX}
  CMAKE_ARGS "${dlib_cmake_args}"
)

# DBow2
#set(dbow2_cmake_args ${common_cmake_args})
#
#list(APPEND dbow2_cmake_args
#  -DBUILD_Demo=OFF
#  )
#
#ExternalProject_Add(DBoW2
#  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/DBoW2-local
#  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/DBoW2/DBoW2-local
#  INSTALL_DIR ${BUILD_INSTALL_PREFIX}
#  CMAKE_ARGS ${dbow2_cmake_args}
#  )

# FBoW
set(fbow_cmake_args ${common_cmake_args})

list(APPEND fbow_cmake_args
  -DBUILD_TESTS=OFF
  -DBUILD_UTILS=OFF
)

ExternalProject_Add(FBoW
  SOURCE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/FBoW/fbow
  BINARY_DIR  ${CMAKE_CURRENT_BINARY_DIR}/FBoW/fbow
  INSTALL_DIR ${BUILD_INSTALL_PREFIX}
  CMAKE_ARGS  ${fbow_cmake_args}
)

# Pangolin; most of the configuration settings disable stuff we don't need
set(pangolin_cmake_args ${common_cmake_args})
list(APPEND pangolin_cmake_args
  -DBUILD_TESTS=NO
  -DBUILD_TOOLS=NO
  -DBUILD_EXAMPLES=NO
  -DBUILD_PANGOLIN_PYTHON=NO
  -DBUILD_PANGOLIN_FFMPEG=NO
  -DBUILD_SHARED_LIBS=NO
  -DBUILD_EXTERN_GLEW=NO
  -DBUILD_EXTERN_LIBPNG=NO
  -DBUILD_EXTERN_LIBJPEG=NO
  -DMSVC_USE_STATIC_CRT=NO
  -DBUILD_PANGOLIN_ZSTD=NO
  -DBUILD_PANGOLIN_LZ4=NO
  -DBUILD_PANGOLIN_LIBOPENEXR=NO
  -DBUILD_PANGOLIN_LIBRAW=NO
  )

ExternalProject_Add(Pangolin
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Pangolin/Pangolin-v0.9.3
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/Pangolin/Pangolin-v0.9.3
  INSTALL_DIR ${BUILD_INSTALL_PREFIX}
  CMAKE_ARGS ${pangolin_cmake_args}
  )
