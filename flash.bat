@echo off
chcp 65001 >nul
REM ============================================================
REM  likong 力控旋钮 一键烧录脚本
REM  用法：双击本文件即可（需要 ST-Link 已接好、板子已供电）
REM  原理：直接用 STM32CubeProgrammer 命令行烧录，
REM        绕开 VS Code 里 ST 扩展的 Flash 加载器报错
REM ============================================================
set CLI=C:\Users\11831\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe
set ELF=%~dp0build\Debug\likong.elf

if not exist "%ELF%" (
    echo [错误] 找不到 %ELF%
    echo 请先在 VS Code 里编译（CMake 构建）成功后，再运行本脚本
    pause
    exit /b 1
)

echo === 开始烧录 %ELF% ... ===
REM 注意 freq=800：杜邦线接 ST-Link 时 4MHz 会擦除失败，降到 800kHz 稳定
"%CLI%" --connect port=SWD mode=UR freq=800 --download "%ELF%" --verify
if errorlevel 1 (
    echo.
    echo [失败] 烧录出错！检查：
    echo   1. ST-Link 是否插好、驱动是否正常
    echo   2. SWD 接线：TMS(PA13)-SWDIO、TCK(PA14)-SWCLK、GND
    echo   3. 板子是否供电
    echo   4. 12V 可以先不接，只留 USB 供电
    pause
    exit /b 1
)

echo.
echo [成功] 烧录并验证通过！正在复位运行...
"%CLI%" --connect port=SWD mode=UR freq=800 -rst
echo 芯片已复位，去串口/OLED 看效果吧。
pause
