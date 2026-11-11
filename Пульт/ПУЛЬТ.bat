@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem Пульт Spotty с 3D-собакой. Адрес робота: ПУЛЬТ.bat --robot 192.168.1.42
where python >nul 2>nul
if errorlevel 1 (py -3 пульт_3d.py %*) else (python пульт_3d.py %*)
if errorlevel 1 pause
