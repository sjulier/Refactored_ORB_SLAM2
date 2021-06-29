PROJECT(ORB-SLAM_Tools)
# Build tools
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Tools)
add_executable(bin_vocabulary bin_vocabulary.cc)
#
target_link_libraries(bin_vocabulary ${PROJECT_NAME} ${DBoW2_LIBS} ORB_SLAM2)
#Ambiguous ORB_SLAM2 project
#include_directories(${CMAKE_SOURCE_DIR}/ThirdParty)
#target_include_directories(bin_vocabulary PUBLIC ${Resources_SOURCE_DIR}/../../Source/Libraries/ORB_SLAM2/include/)