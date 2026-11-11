@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem Выставочный показ Spotty. N - следующая сцена, Q - выход.
python show.py %*
if errorlevel 1 pause
