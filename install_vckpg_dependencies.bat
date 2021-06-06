@echo off 

call Scripts\ValidateVCPKG.bat

if not "%errorlevel%"=="0" (
  exit /b %errorlevel%
)

%VCPKG_ROOT%\vcpkg install boost eigen3 suitesparse opencv4
