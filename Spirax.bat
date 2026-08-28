@echo off
title Spirax HMI

rem --- auto-elevacion: si no somos admin, nos relanzamos pidiendo UAC ---
net session >nul 2>&1
if errorlevel 1 (
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

cd /d "%~dp0"

rem Mata cualquier instancia previa: un python huerfano deja el SMBus
rem tomado y la HMI nueva falla con codigo 3.
taskkill /F /IM python.exe >nul 2>&1
del "%TEMP%\spirax_hmi.lock" >nul 2>&1
timeout /t 2 /nobreak >nul

call venv\Scripts\activate.bat
python hmi.py

if errorlevel 1 (
    echo.
    echo === La HMI termino con error. Revisa el mensaje de arriba. ===
    pause
)