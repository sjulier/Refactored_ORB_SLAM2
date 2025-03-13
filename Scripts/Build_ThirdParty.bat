rem Check if the file exists which tells us that the third party has been built

set third_party_build_tag=%cd%\Develop\%build_type%\3rd_party_built_tag.txt

if exist "%third_party_build_tag%" (
    echo .
    echo Third party libraries built already; skipping
    echo .
    exit /b 0
)

echo Build type %build_type%
set root_dir="%cd%"
mkdir "Build/%build_type%/Source/ThirdParty"
pushd "Build/%build_type%/Source/ThirdParty"
cmake.exe "%root_dir%\Source\ThirdParty" -DCMAKE_BUILD_TYPE=%build_type% -G"Ninja" -DCMAKE_TOOLCHAIN_FILE="%toolchain_file%" -DVCPKG_INSTALLED_DIR=%vcpkg_installed_dir%
set thirdpartyerrorlevel=%errorlevel%

if  %thirdpartyerrorlevel% == 0 (
    cmake --build .
    set thirdpartyerrorlevel=%errorlevel%
)
if  %thirdpartyerrorlevel% == 0 (
    cmake --install .
    set thirdpartyerrorlevel=%errorlevel%
)

popd

echo thirdpartyerrorlevel=%thirdpartyerrorlevel%

rem Add the empty file which confirms the third party libraries have been built; only do this if nmake returned no error
if %thirdpartyerrorlevel% == 0 (type nul > "%third_party_build_tag%")
