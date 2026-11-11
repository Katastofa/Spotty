@echo off
chcp 65001 >nul
rem Пульт Spotty с 3D-собакой. Сам пульт — в папке Пульт.
call "%~dp0Пульт\ПУЛЬТ.bat" %*
