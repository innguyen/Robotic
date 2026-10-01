@echo off
rem Run once to upload enrollment and compatible models. Not needed on every startup.
scp -r "%~dp0face_db" "%~dp0face_models" jetauto@192.168.149.1:/home/jetauto/
if errorlevel 1 exit /b 1
echo Face assets uploaded. Run run_smart_search.bat to start.
