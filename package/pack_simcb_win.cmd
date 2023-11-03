@echo off
setlocal

:: Work in same directory as this script
cd "%~dp0"

:: Setup base directory structure
rmdir /s /q pack\simcb\win-x64\
mkdir pack\simcb\win-x64\

:: Build firmware
cd ..\firmware
rmdir /s /q build\
cmake --preset simcb-win
cmake --build --preset simcb-win-release
cd ..\package\

:: Copy firmware files
copy ..\firmware\build\simcb-win\Release\SimCB.exe pack\simcb\win-x64\
