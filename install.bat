@echo off
rem Installs the tools the project needs. Double-click this file once; safe to run again.
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\install-windows.ps1" %*
pause
