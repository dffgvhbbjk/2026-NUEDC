@echo off
setlocal

set "PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%PY%" goto run

set "PY=%LOCALAPPDATA%\Python\pythoncore-3.14-64\python.exe"
if exist "%PY%" goto run

set "PY="
for /f "delims=" %%P in ('where py 2^>nul') do if not defined PY set "PY=%%P"
if not defined PY for /f "delims=" %%P in ('where python 2^>nul') do if not defined PY set "PY=%%P"
if not defined PY goto no_python

:run
cd /d "%~dp0.."
echo Starting pile inspection web app...
echo Open http://localhost:5000 in your browser.
"%PY%" "%~dp0web_app.py" --host 0.0.0.0 --port 5000
pause
exit /b %ERRORLEVEL%

:no_python
echo Python runtime not found.
echo Install Python 3.12 or newer, then run this file again.
pause
exit /b 1
