@echo off
REM Build the Docker image for husky-assembly-teleop

echo Updating git submodules...
cd /d "%~dp0.."
git submodule update --init --recursive

echo.
echo Building husky-assembly-teleop Docker image...
docker compose build husky-teleop-dev

echo.
echo Build complete!
echo.
echo To start the container, run: docker\run.bat
pause
