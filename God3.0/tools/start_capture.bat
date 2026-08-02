@echo off
rem ============================================================================
rem 采集启动器 -- 双击运行, 自动用真实 Python 路径 (绕开商店存根)
rem ============================================================================
chcp 65001 >nul
setlocal

set "PY=C:\Users\31368\AppData\Local\Python\bin\python.exe"
if not exist "%PY%" (
    echo [错误] 找不到 Python: %PY%
    echo 请修改本文件顶部的 PY 路径
    pause
    exit /b 1
)

rem 检查 pyserial, 没装就自动装
"%PY%" -c "import serial" 2>nul
if errorlevel 1 (
    echo 首次运行, 正在安装 pyserial ...
    "%PY%" -m pip install pyserial
)

set /p COMPORT=串口号 (直接回车默认 COM5): 
if "%COMPORT%"=="" set COMPORT=COM5

"%PY%" "%~dp0..\..\pc_tools\capture\uart_frame_parser.py" --port %COMPORT% --outdir "%~dp0..\data"

echo.
pause
