@echo off
rem Explicit live-mode launcher. Navigation still starts paused on the web page.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_test_robot.ps1" -Mode live %*
set "RESULT=%ERRORLEVEL%"
echo.
pause
exit /b %RESULT%
