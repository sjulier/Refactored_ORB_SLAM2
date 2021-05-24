add_executable(mono_tum mono_tum.cc)
target_link_libraries(mono_tum ORB_SLAM2)

add_executable(mono_euroc mono_euroc.cc)
target_link_libraries(mono_euroc ORB_SLAM2)

add_executable(mono_kitti mono_kitti.cc)
target_link_libraries(mono_kitti ORB_SLAM2)
