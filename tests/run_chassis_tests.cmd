@echo off
rem @file run_chassis_tests.cmd
rem @brief 使用主机 GCC 构建并运行底盘运动测试。
rem @board 底盘板
rem @vehicle_function 核对运动模式、坐标变换和四轮分配结果。
rem @inputs 运动控制源码、四轮运动学源码和原有测试用例。
rem @outputs 测试可执行文件、断言结果和退出码。
rem @dependencies GCC、数学库和 Windows 命令解释器。
rem @note 重构说明：本文件行为与原代码保持一致。
setlocal
set "CHASSIS_TEST_ROOT=%~dp0.."
set "CHASSIS_TEST_OUT=%CHASSIS_TEST_ROOT%\build\host-tests"
if not exist "%CHASSIS_TEST_OUT%" mkdir "%CHASSIS_TEST_OUT%"
if errorlevel 1 exit /b 1
gcc -std=c11 -Wall -Wextra -Werror -pedantic -UNDEBUG -I "%CHASSIS_TEST_ROOT%\Core\Inc" "%CHASSIS_TEST_ROOT%\Core\Src\chassis_motion_control.c" "%CHASSIS_TEST_ROOT%\Core\Src\chassis_wheel_kinematics.c" "%CHASSIS_TEST_ROOT%\tests\chassis_motion_test.c" -o "%CHASSIS_TEST_OUT%\chassis_motion_test.exe" -lm
if errorlevel 1 exit /b 1
"%CHASSIS_TEST_OUT%\chassis_motion_test.exe"
exit /b %errorlevel%
