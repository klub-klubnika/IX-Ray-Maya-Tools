@echo off
cmake --preset vs2022
if errorlevel 1 exit /b %errorlevel%

echo.
echo Solution created: build-vs2022\IXRayMayaTools.sln
