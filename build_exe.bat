@echo off
echo ===============================================
echo  apt.dat to Blender Layout Tool - EXE builder
echo ===============================================
echo.
echo Installing required Python packages (earcut, numpy, pyinstaller)...
pip install -r requirements.txt
if errorlevel 1 (
    echo.
    echo Something went wrong installing packages. Make sure Python and pip
    echo are installed and on your PATH, then try again.
    pause
    exit /b 1
)

echo.
echo Building AptDatToBlender.exe ...
pyinstaller --onefile --windowed --name AptDatToBlender aptdat_app.py

echo.
echo ===============================================
echo Done! Your exe is in the "dist" folder:
echo   dist\AptDatToBlender.exe
echo You can copy that single file anywhere and run it directly.
echo ===============================================
pause
