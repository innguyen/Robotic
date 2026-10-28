@echo off
rem One-time: install an SSH key so the robot launchers stop asking for the password.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup_ssh_key.ps1" %*
set "RESULT=%ERRORLEVEL%"
echo.
pause
exit /b %RESULT%
