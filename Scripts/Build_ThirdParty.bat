rem Check if the file exists which tells us that the third party has been built

set third_party_build_tag=%cd%\Develop\%build_type%\3rd_party_built_tag.txt

if exist %third_party_build_tag% (
    echo .
    echo Third party libraries built already; skipping
    echo .
    exit /b 0
)

echo Build type %build_type%
set root_dir=%cd%
mkdir "Build/%build_type%/Source/ThirdParty"
pushd "Build/%build_type%/Source/ThirdParty"
cmake.exe %root_dir%\Source\ThirdParty -DCMAKE_BUILD_TYPE=%build_type% -G"Visual Studio 16 2019" -DCMAKE_TOOLCHAIN_FILE=%toolchain_file%
msbuild ALL_BUILD.vcxproj -maxcpucount:4
popd

rem Add the empty file which confirms the third party libraries have been built; only do this if nmake returned no error
if %errorlevel% == 0 (type nul > %third_party_build_tag%)
