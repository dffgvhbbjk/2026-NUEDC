@echo off
REM ============================================================================
REM run_i2s_sim.bat — 双击运行 PCM1808 I2S 仿真 (Windows)
REM
REM 前提: ModelSim 或 ModelSim-Altera 已加入 PATH 环境变量
REM   - 独立 ModelSim: 安装目录\win64 或 win32 应在 PATH
REM   - Quartus 自带:  <Quartus安装目录>\modelsim_ase\win32aloader
REM                   或 <Quartus安装目录>\modelsim_ae\win64
REM
REM 如果 vsim 命令找不到, 请手动设置 MODELSIM_PATH 环境变量指向 modelsim_ase 目录
REM ============================================================================

setlocal
cd /d "%~dp0\.."

echo ==========================================
echo PCM1808 I2S Receiver Simulation
echo Working dir: %CD%
echo ==========================================

REM ---- 检查 vsim 是否可用 ----
where vsim >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo.
    echo ERROR: vsim not found in PATH
    echo.
    echo Please choose one of:
    echo   1. Add ModelSim to PATH
    echo   2. Set MODELSIM_PATH env var, e.g.:
    echo      set MODELSIM_PATH=C:\altera\13.0sp1\modelsim_ase
    echo      (then run this script again)
    echo.
    echo   3. Open ModelSim manually and run:
    echo      do sim/run_i2s_sim.do
    echo.
    pause
    exit /b 1
)

REM ---- 运行无 GUI 自检，结束后自动返回批处理窗口 ----
vsim -c -do sim/run_i2s_sim_batch.do

echo.
echo ==========================================
echo Simulation finished. Check output above.
echo ==========================================
pause
