@echo off
rem One tap: starts the input server and the Arm Hub, opens the robot viewer. Output starts SAFE.
rem The link for your phone is printed below (same WiFi).
cd /d "%~dp0"
set "PY=%USERPROFILE%\anaconda3\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" launch.py
pause
