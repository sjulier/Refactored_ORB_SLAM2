echo Build type %build_type%
set root_dir=%cd%
mkdir "Build/%build_type%/Source/ThirdParty"
pushd "Build/%build_type%/Source/ThirdParty"
cmake.exe %root_dir%\Source\ThirdParty -DCMAKE_BUILD_TYPE=%build_type% -G"NMake Makefiles" -DCMAKE_TOOLCHAIN_FILE=%toolchain_file%
nmake
popd
