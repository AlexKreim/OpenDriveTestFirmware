@echo off
cd /d "%~dp0"
if not exist .venv\Scripts\activate.bat (
  rem 'py' launcher if installed, otherwise plain 'python' (Microsoft Store / python.org without the launcher)
  py -3 -m venv .venv 2>nul || python -m venv .venv
  if not exist .venv\Scripts\activate.bat (
    echo Could not create .venv: install Python 3 from python.org and tick "Add python.exe to PATH".
    pause
    exit /b 1
  )
  call .venv\Scripts\activate.bat
  python -m pip install --upgrade pip
  pip install -r requirements.txt
) else (
  call .venv\Scripts\activate.bat
)
python main.py
pause
