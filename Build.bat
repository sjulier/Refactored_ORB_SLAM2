@echo off

rem See if cmake can be found. If this fails, first try a fallback (default position)
rem If the fallback fails, report an error and die.

cmake --version >NUL 2>NUL

if errorlevel 1 (
    echo Cannot find cmake.exe on the command line; calling default vsvarsall.bat to install
    call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
    cmake --version >NUL 2>NUL
)

if errorlevel 1 (
    echo.
    echo Cannot find cmake.exe; check that the cmake command line extensions are installed and
    echo run vcvarsall.bat to register the build system
    echo.
    echo An example command line is:
    echo.
    echo "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
    echo.
    exit /b 1
)

rem Now check that VCPKG is installed

if "%VCPKG_ROOT%"=="" (
    echo.
    echo Please set VCPKG_ROOT to the vcpkg installation directory.
    echo.
    exit /b 1
)

rem Set the build type

if "%1" == "" (
    set build_type=Debug
) else (
    set build_type=%1
)

rem Check the build type is valid

if not "%build_type%" == "Debug" (
    if not "%build_type%" == "Release" (
        echo.
        echo Unknown build type %build_type%
        echo Supported types are Debug or Release
        echo.
        exit /b 1
    )
)

rem Create the sysexits.h file if it doesn't exist
set sysexits_file=Develop\%build_type%\include\sysexits.h
echo sysexits_file=%sysexits_file%
if not exist "%sysexits_file%\include" (
	mkdir Develop\%build_type%\include >NUL 2>NUL
	echo #ifndef _SYSEXITS_H_ > %sysexits_file%
	echo #define _SYSEXITS_H_ >> %sysexits_file%
	echo #define EX_OK           0       >> %sysexits_file%
	echo #define EX__BASE        64      >> %sysexits_file%
	echo #define EX_USAGE        64      >> %sysexits_file%
	echo #define EX_DATAERR      65      >> %sysexits_file%
	echo #define EX_NOINPUT      66      >> %sysexits_file%
	echo #define EX_NOUSER       67      >> %sysexits_file%
	echo #define EX_NOHOST       68      >> %sysexits_file%
	echo #define EX_UNAVAILABLE  69      >> %sysexits_file%
	echo #define EX_SOFTWARE     70      >> %sysexits_file%
	echo #define EX_OSERR        71      >> %sysexits_file%
	echo #define EX_OSFILE       72      >> %sysexits_file%
	echo #define EX_CANTCREAT    73      >> %sysexits_file%
	echo #define EX_IOERR        74      >> %sysexits_file%
	echo #define EX_TEMPFAIL     75      >> %sysexits_file%
	echo #define EX_PROTOCOL     76      >> %sysexits_file%
	echo #define EX_NOPERM       77      >> %sysexits_file%
	echo #define EX_CONFIG       78      >> %sysexits_file%
	echo #define EX__MAX 78      >> %sysexits_file%
	echo #endif >> %sysexits_file%
)


rem Set up the tool chain file which is needed
set toolchain_file=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake

rem Now call the build scripts
call Build_ThirdParty.bat %build_type% %toolchain_file%
call Build_Source.bat %build_type% %toolchain_file%