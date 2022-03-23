
# The default directory for the settings
set(MONO_DEFAULT_SETTINGS_DIR "${DEFAULT_ORB_SETTINGS_DIR}/Monocular/")

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
   add_compile_definitions(DEFAULT_MONO_SETTINGS_DIR="${MONO_DEFAULT_SETTINGS_DIR}")
else()
   add_definitions(-DDEFAULT_MONO_SETTINGS_DIR="${MONO_DEFAULT_SETTINGS_DIR}")
endif()

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

# Install the yaml settings files
install(TARGETS mono_tum mono_euroc mono_kitti RUNTIME DESTINATION ${BUILD_INSTALL_PREFIX}/bin)
file(GLOB settings_files "*.yaml")
install(FILES ${settings_files} DESTINATION ${MONO_DEFAULT_SETTINGS_DIR})

# Install the timestamps, although there's actually no evidence they are used
install(DIRECTORY EuRoC_TimeStamps DESTINATION ${MONO_DEFAULT_SETTINGS_DIR})
