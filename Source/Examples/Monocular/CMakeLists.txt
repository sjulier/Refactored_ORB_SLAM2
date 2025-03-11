# The default directory for the settings
cmake_path(SET MONO_DEFAULT_SETTINGS_DIR NORMALIZE "${DEFAULT_ORB_SETTINGS_DIR}/Monocular/")

add_compile_definitions(DEFAULT_MONO_SETTINGS_DIR="${MONO_DEFAULT_SETTINGS_DIR}")

# TUM
add_executable(mono_tum mono_tum.cc)
target_link_libraries(mono_tum ORB_SLAM2)
set_target_properties(mono_tum PROPERTIES OUTPUT_NAME mono_tum${EXE_POSTFIX})

# EuRoC
add_executable(mono_euroc mono_euroc.cc)
target_link_libraries(mono_euroc ORB_SLAM2)
set_target_properties(mono_euroc PROPERTIES OUTPUT_NAME mono_euroc${EXE_POSTFIX})

# KITTI
add_executable(mono_kitti mono_kitti.cc)
target_link_libraries(mono_kitti ORB_SLAM2)
set_target_properties(mono_kitti PROPERTIES OUTPUT_NAME mono_kitti${EXE_POSTFIX})

# Install the compiled targets
install(TARGETS mono_tum mono_euroc mono_kitti RUNTIME DESTINATION ${BUILD_INSTALL_PREFIX}/bin)

# Install the yaml settings files
file(GLOB settings_files "*.yaml")
install(FILES ${settings_files} DESTINATION ${MONO_DEFAULT_SETTINGS_DIR})

# Install the EuRoC timestamps
install(DIRECTORY EuRoC_TimeStamps DESTINATION ${MONO_DEFAULT_SETTINGS_DIR})
