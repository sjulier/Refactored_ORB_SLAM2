cmake_minimum_required(VERSION 3.1)

project(Resources)

set(BUILD_INSTALL_PREFIX ${Resources_SOURCE_DIR}/../../Develop/${CMAKE_BUILD_TYPE})

add_subdirectory(Vocabulary)


# Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools)
add_executable(bin_vocabulary
        Tools/bin_vocabulary.cc)

target_link_libraries(bin_vocabulary ${PROJECT_NAME} ${DBoW2_LIBS})

target_include_directories(bin_vocabulary PUBLIC ${Resources_SOURCE_DIR}/../../Source/Libraries/ORB_SLAM2/include/)