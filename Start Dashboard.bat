@echo off
rem Starts the Arm Hub dashboard (http://127.0.0.1:8765). The output starts SAFE.
cd /d "%~dp0"
set "PY=%USERPROFILE%\anaconda3\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" dashboard\hub.py
pause
