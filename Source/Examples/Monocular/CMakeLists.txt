
# The default directory for the settings
set(MONO_DEFAULT_SETTINGS_DIR "${DEFAULT_ORB_SETTINGS_DIR}/Monocular/")

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
   add_compile_definitions(DEFAULT_MONO_SETTINGS_DIR="${MONO_DEFAULT_SETTINGS_DIR}")
else()
   add_definitions(-DDEFAULT_MONO_SETTINGS_DIR="${MONO_DEFAULT_SETTINGS_DIR}")
endif()

add_executable(mono_tum mono_tum.cc)
target_link_libraries(mono_tum ORB_SLAM2)

add_executable(mono_euroc mono_euroc.cc)
target_link_libraries(mono_euroc ORB_SLAM2)

add_executable(mono_kitti mono_kitti.cc)
target_link_libraries(mono_kitti ORB_SLAM2)

install(TARGETS mono_tum mono_euroc mono_kitti RUNTIME DESTINATION ${BUILD_INSTALL_PREFIX}/bin)
file(GLOB settings_files "*.yaml")
install(FILES ${settings_files} DESTINATION ${MONO_DEFAULT_SETTINGS_DIR})
