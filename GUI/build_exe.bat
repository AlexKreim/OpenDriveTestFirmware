@echo off
cd /d "%~dp0"
if not exist .venv (
  py -3 -m venv .venv 2>nul || python -m venv .venv
)
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt pyinstaller
pyinstaller --noconfirm --windowed --name ESC_Control --add-data "presets;presets" --hidden-import serial.tools.list_ports main.py
echo.
echo Done: dist\ESC_Control\ESC_Control.exe
pause
