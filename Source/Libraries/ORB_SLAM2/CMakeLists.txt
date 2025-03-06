# For debugging strip the full path from https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
# Note this will not work in Visual studio-type developer environments.

if(NOT DEFINED MSVC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-builtin-macro-redefined -D'__FILE__=\"$(subst $(realpath ${CMAKE_SOURCE_DIR})/,,$(abspath $<))\"'")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-builtin-macro-redefined -D'__FILE__=\"$(subst $(realpath ${CMAKE_SOURCE_DIR})/,,$(abspath $<))\"'")
endif()

if(NOT DEFINED BUILD_SHARED_LIBRARIES)
  set(BUILD_SHARED_LIBRARIES NO)
endif()

if(BUILD_SHARED_LIBRARIES)
  set(LIB_TYPE SHARED)
else()
  set(LIB_TYPE STATIC)
endif()

project(ORB_SLAM2)

add_library(${PROJECT_NAME} ${LIB_TYPE}
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/ORBVocabulary.cc
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

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_DIRECTORY} ${DBoW2_INCLUDE_DIR} ${DLib_INCLUDE_DIR})

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
target_compile_definitions(${PROJECT_NAME} PUBLIC HAVE_GLEW)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
