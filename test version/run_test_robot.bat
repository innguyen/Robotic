@echo off
rem Double-click for preview. Optional: run_test_robot.bat -Mode live
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_test_robot.ps1" %*
set "RESULT=%ERRORLEVEL%"
echo.
pause
exit /b %RESULT%
