@echo off
cd /d "%~dp0"
python -c "import numpy, tkinter" >nul 2>&1
if errorlevel 1 (
  echo Requires Python with tkinter and numpy. Install numpy: python -m pip install numpy
  pause
  exit /b 1
)
python simulate_navigation.py
if errorlevel 1 pause
