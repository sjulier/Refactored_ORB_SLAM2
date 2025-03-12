# The default directory for the settings
cmake_path(SET RGBD_DEFAULT_SETTINGS_DIR NORMALIZE "${DEFAULT_ORB_SETTINGS_DIR}/RGB-D/")
add_compile_definitions(DEFAULT_RGBD_SETTINGS_DIR="${RGBD_DEFAULT_SETTINGS_DIR}")

# TUM
add_executable(rgbd_tum rgbd_tum.cc)
target_link_libraries(rgbd_tum ORB_SLAM2)
set_target_properties(rgbd_tum PROPERTIES OUTPUT_NAME rgbd_tum${EXE_POSTFIX})

# Install compiled target
install(TARGETS rgbd_tum RUNTIME DESTINATION ${BUILD_INSTALL_PREFIX}/bin)

# Install the yaml settings files
file(GLOB settings_files "*.yaml")
install(FILES ${settings_files} DESTINATION ${RGBD_DEFAULT_SETTINGS_DIR})

# Install the associations
install(DIRECTORY associations DESTINATION ${RGBD_DEFAULT_SETTINGS_DIR})
