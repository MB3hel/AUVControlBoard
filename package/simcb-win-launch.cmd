@echo off
setlocal
cd "%~dp0"

set has_port=0
:promptport
set /p "port=TCP Port for SimCB: "
if %has_port%==0 goto promptport
