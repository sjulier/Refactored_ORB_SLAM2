PROJECT(ORB-SLAM_Vocabulary)

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.18.0")
  add_custom_command(OUTPUT ORBvoc.txt file(ARCHIVE_EXTRACT INPUT ${CMAKE_CURRENT_SOURCE_DIR}/ORBvoc.txt.tar.gz))
else()
  add_custom_command(OUTPUT ORBvoc.txt COMMAND ${CMAKE_COMMAND} -E tar xvf ${CMAKE_CURRENT_SOURCE_DIR}/ORBvoc.txt.tar.gz)
endif()

add_custom_target(orb_vocabulary ALL WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DEPENDS ORBvoc.txt)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.txt DESTINATION ${BUILD_INSTALL_PREFIX}/var/lib/orbslam2)
