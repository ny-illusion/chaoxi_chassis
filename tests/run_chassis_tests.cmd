@echo off
setlocal
set "CHASSIS_TEST_ROOT=%~dp0.."
set "CHASSIS_TEST_OUT=%CHASSIS_TEST_ROOT%\build\host-tests"
if not exist "%CHASSIS_TEST_OUT%" mkdir "%CHASSIS_TEST_OUT%"
if errorlevel 1 exit /b 1
gcc -std=c11 -Wall -Wextra -Werror -pedantic -UNDEBUG -I "%CHASSIS_TEST_ROOT%\Core\Inc" "%CHASSIS_TEST_ROOT%\Core\Src\chassis_motion.c" "%CHASSIS_TEST_ROOT%\tests\chassis_motion_test.c" -o "%CHASSIS_TEST_OUT%\chassis_motion_test.exe" -lm
if errorlevel 1 exit /b 1
"%CHASSIS_TEST_OUT%\chassis_motion_test.exe"
exit /b %errorlevel%
