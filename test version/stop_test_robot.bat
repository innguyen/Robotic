@echo off
rem Stop the test stack on the robot (vision, navigation, Collision Monitor).
rem Sensor drivers, chassis and roscore keep running.
ssh -o ConnectTimeout=10 jetauto@192.168.149.1 "[ -f /home/jetauto/test_version/stop_test_stack_ros1.sh ] && sed -i 's/\r$//' /home/jetauto/test_version/stop_test_stack_ros1.sh && bash /home/jetauto/test_version/stop_test_stack_ros1.sh || echo 'Stop script not on robot yet: run run_test_robot.bat once first.'"
set "RESULT=%ERRORLEVEL%"
echo.
pause
exit /b %RESULT%
