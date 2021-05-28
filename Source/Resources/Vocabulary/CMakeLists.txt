if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.18.0")
  add_custom_command(OUTPUT ORBvoc.txt file(ARCHIVE_EXTRACT INPUT ORBvoc.txt.gz))
else()
  add_custom_command(OUTPUT ORBvoc.txt COMMAND ${CMAKE_COMMAND} -E gunzip ORBvoc.txt.gz)
endif()

add_custom_target(unpack_vocabulary DEPENDS ORBvoc.txt)

install(FILES ORBvoc.txt DESTINATION ${BUILD_INSTALL_PREFIX}/bin)

