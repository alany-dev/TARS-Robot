@echo off
chcp 936 >nul 2>nul
setlocal
:: 只追加需要的工具路径，不要把整条命令写一行
set "TOOL_PATH=C:\Users\25725\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin;C:\Users\25725\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin;E:\Program Files\CMake\bin"
set "PATH=%TOOL_PATH%;%PATH%"

set BUILD_DIR=e:/Cpp_Projects/TARS-Robot/mcu/prj/build/Debug
set ELF_FILE=%BUILD_DIR%/prj.elf
set PROG_CLI=C:/Users/25725/AppData/Local/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI.exe

echo === CMake Build ===
cmake --build "%BUILD_DIR%"
if errorlevel 1 (
    echo [STM32-RESULT] FAIL
    exit /b 1
)

echo === Flash & Reset ===
"%PROG_CLI%" -c port=SWD -w "%ELF_FILE%" -v -rst
if errorlevel 1 (
    echo [STM32-RESULT] FAIL
    exit /b 1
)

echo [STM32-RESULT] OK
endlocal
