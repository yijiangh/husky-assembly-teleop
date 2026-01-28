@echo off
REM Run the husky-assembly-teleop container on Windows

cd /d "%~dp0.."

echo Starting husky-assembly-teleop container...
echo.
echo NOTE: For GUI (pybullet), start VcXsrv with "Disable access control" first
echo.

docker compose run --rm husky-teleop-windows
