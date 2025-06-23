
if(NOT DEFINED BUILD_SHARED_LIBRARIES)
  set(BUILD_SHARED_LIBRARIES NO)
endif()

if(BUILD_SHARED_LIBRARIES)
  set(LIB_TYPE SHARED)
else()
  set(LIB_TYPE STATIC
          include/AKAZEextractor.h
          src/AKAZEextractor.cc)
endif()

project(ORB_SLAM2)

add_library(${PROJECT_NAME} ${LIB_TYPE}
src/Associater.cc
src/Converter.cc
src/FeatureExtractor.cc
src/FeatureExtractorFactory.cc
src/FeaturePoint.cc
src/Frame.cc
src/FrameDrawer.cc
src/Initializer.cc
src/KeyFrame.cc
src/KeyFrameDatabase.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/Map.cc
src/MapDrawer.cc
src/MapPoint.cc
src/Optimizer.cc
src/ORBextractor.cc
src/ORBVocabulary.cc
src/PnPsolver.cc
src/Sim3Solver.cc
src/System.cc
src/Tracking.cc
src/Viewer.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_DIRECTORY})
target_link_libraries(${PROJECT_NAME} PUBLIC

${OpenCV_LIBS}
Eigen3::Eigen
pango_opengl
pango_display
${DBoW2_LIBS}
${DLib_LIBS}
g2o::types_sba
g2o::types_sim3
g2o::core
Boost::filesystem
)

# For some reason this isn't propagating over; I think this might be related to a LIST issue,
# but I'm not sure
if(APPLE)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAVE_GLEW)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${DBoW2_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${DLib_INCLUDE_DIR}>
  )
