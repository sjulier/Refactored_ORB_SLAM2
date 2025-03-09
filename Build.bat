@echo off

call Scripts\ValidateVCPKG.bat

if not "%errorlevel%"=="0" (
  exit /b %errorlevel%
)

rem Set the build type

if "%1" == "" (
    set build_type=Release
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
if not exist "%sysexits_file%" (
        echo Creating sysexits.h file %sysexits_file%
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

rem Set the local for the vcpkg install

set vcpkg_installed_dir=%~dp0vcpkg_installed

rem Now call the build scripts
call Scripts\Build_ThirdParty.bat %build_type% "%toolchain_file%" "%vcpkg_installed_dir%"
call Scripts\Build_Source.bat %build_type% "%toolchain_file%" "%vcpkg_installed_dir%"

rem We need to somehow get these into the system path
rem echo %~dp0\Install\bin
rem echo %VCPKG_ROOT%\installed\x64-windows\bin
