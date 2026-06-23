@echo off
REM ============================================================
REM SBR 几何寻径模块 — 自测试脚本
REM 用法: selftest.bat [Release|Debug]
REM ============================================================
setlocal enabledelayedexpansion

set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=Release

set BUILD_DIR=build
set APP=%BUILD_DIR%\%CONFIG%\sbr_app.exe
set TEST_QUICK=%BUILD_DIR%\%CONFIG%\sbr_tests_quick.exe

echo ============================================================
echo  SBR Self-Test Suite
echo  Config: %CONFIG%
echo ============================================================

REM ── Step 1: Build ──
echo.
echo [1/3] Building...
cmake -B %BUILD_DIR% -G "Visual Studio 17 2022" -A x64 >nul 2>&1
if errorlevel 1 (
    echo   FAILED: CMake configure error
    exit /b 1
)
cmake --build %BUILD_DIR% --config %CONFIG% --target sbr_tests_quick sbr_app >nul 2>&1
if errorlevel 1 (
    echo   FAILED: Build error
    exit /b 1
)
echo   OK

REM ── Step 2: Unit Tests ──
echo.
echo [2/3] Running unit tests...
set FAILED=0
%TEST_QUICK% 2>&1 | findstr /C:"passed" /C:"failed"
if errorlevel 1 set FAILED=1
if %FAILED%==1 (
    echo   FAILED: Unit tests
    exit /b 1
)
echo   OK

REM ── Step 3: Demo Scene ──
echo.
echo [3/3] Running demo scene...
set OUTPUT=
for /f "tokens=*" %%a in ('%APP% demo\Scene\meeting.obj demo\Material\material_map-meeting.json 2^>^&1 ^| findstr /C:"Status:" /C:"Total paths:" /C:"Trace time:"') do set OUTPUT=!OUTPUT! %%a
echo   !OUTPUT!
echo   OK

echo.
echo ============================================================
echo  Self-test complete!
echo ============================================================
exit /b 0
