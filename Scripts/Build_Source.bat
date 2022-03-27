rem Check if the vocabulary file exists. If not, uncompress it

if not exist %~dp0\..\Source\Vocabulary\ORBvoc.txt (

   powershell -command "Expand-Archive -Force %~dp0..\Source\Vocabulary\ORBvoc.txt.zip %~dp0..\Source\Vocabulary"

)

rem Build

echo Build type %build_type%
set root_dir=%cd%
mkdir "Build/%build_type%/Source/ORBSLAM_2"
pushd "Build/%build_type%/Source/ORBSLAM_2"
cmake.exe %root_dir%\Source -DCMAKE_BUILD_TYPE=%build_type% -G"Visual Studio 16 2019" -DCMAKE_TOOLCHAIN_FILE=%toolchain_file% -DCMAKE_C_FLAGS="-bigoj" -DCMAKE_CXX_FLAGS="-bigobj -EHsc"
msbuild INSTALL.vcxproj -maxcpucount:4 -property:Configuration=%build_type%
popd

rem Copy the DLLs over afterwards because the install target doesn't
copy Build\%build_type%\Source\ORBSLAM_2\Examples\Monocular\%build_type%\*.dll Install\bin
