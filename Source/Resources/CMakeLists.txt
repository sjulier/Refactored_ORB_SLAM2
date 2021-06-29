cmake_minimum_required(VERSION 3.1)

project(Resources)

set(BUILD_INSTALL_PREFIX ${Resources_SOURCE_DIR}/../../Develop/${CMAKE_BUILD_TYPE})

add_subdirectory(Vocabulary)

add_subdirectory(Tools)
