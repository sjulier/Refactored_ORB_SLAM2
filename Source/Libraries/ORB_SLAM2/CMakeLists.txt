include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

project(ORB_SLAM2)

message(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
Eigen3::Eigen
pangolin
DBoW2::DBoW2
g2o::types_sba
g2o::types_sim3
g2o::core
)


target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
