@echo off
rem Builds the game and starts it. Double-click this file.
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" -Run -SkipTests
if errorlevel 1 pause
