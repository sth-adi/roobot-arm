@echo off
rem Stops old copies of the app, pulls the latest code (branch rarm-core) and starts it. The phone link is printed when it is up.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0update_and_start.ps1"
pause
