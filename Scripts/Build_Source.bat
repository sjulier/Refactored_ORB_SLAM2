echo Build type %build_type%
set root_dir=%cd%
mkdir "Build/%build_type%/Source/ORBSLAM_2"
pushd "Build/%build_type%/Source/ORBSLAM_2"
cmake.exe %root_dir%\Source -DCMAKE_BUILD_TYPE=%build_type% -G"NMake Makefiles" -DCMAKE_TOOLCHAIN_FILE=%toolchain_file% -DCMAKE_C_FLAGS="-bigoj" -DCMAKE_CXX_FLAGS="-bigobj"
nmake
popd
