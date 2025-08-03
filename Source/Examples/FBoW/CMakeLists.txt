find_package(OpenMP REQUIRED)

add_executable(train_fbow train_fbow.cpp)

target_link_libraries(train_fbow
        fbow
        ${OpenCV_LIBS}
        ORB_SLAM2
        OpenMP::OpenMP_CXX
)

target_include_directories(train_fbow PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/Source/Libraries/ORB_SLAM2/include
)

set_target_properties(train_fbow
        PROPERTIES OUTPUT_NAME train_fbow${EXE_POSTFIX})

install(TARGETS train_fbow RUNTIME DESTINATION ${BUILD_INSTALL_PREFIX}/bin)