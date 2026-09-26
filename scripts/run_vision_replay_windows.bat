@echo off
setlocal

pushd "%~dp0"
if errorlevel 1 (
    echo Could not open the repository folder.
    pause
    exit /b 1
)

set "VENV=%LOCALAPPDATA%\MosasGuidanceDart\vision-replay"
set "PYTHON=%VENV%\Scripts\python.exe"

if not exist "%PYTHON%" (
    py -3.13 -m venv "%VENV%"
    if errorlevel 1 goto environment_error
)

"%PYTHON%" -c "import cv2, numpy" >nul 2>&1
if errorlevel 1 (
    echo Preparing the Windows vision replay environment...
    "%PYTHON%" -m pip install --disable-pip-version-check opencv-python numpy
    if errorlevel 1 goto package_error
)

"%PYTHON%" scripts\vision_replay.py
set "RESULT=%ERRORLEVEL%"
popd
if not "%RESULT%"=="0" (
    echo Vision replay exited with code %RESULT%.
    pause
)
exit /b %RESULT%

:environment_error
echo Could not create the Python 3.13 environment. Install Python 3.13 with the Windows Python Launcher.
pause
popd
exit /b 1

:package_error
echo Could not install OpenCV and NumPy. Check the network connection and try again.
pause
popd
exit /b 1
