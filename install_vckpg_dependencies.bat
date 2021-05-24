if "%VCKPG_ROOT%"=="" (
    echo.
    echo Please set VCKPG_ROOT to the vcpkg installation directory.
    echo.
    exit /b 1
)

%VCKPG_ROOT%\vcpkg install boost:x64-windows eigen3:x64-windows suitesparse:x64-windows
