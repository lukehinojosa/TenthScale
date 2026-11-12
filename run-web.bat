@echo off
rem Builds the browser version and opens it. Close this window to stop it.
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" -Web -Run
if errorlevel 1 pause
