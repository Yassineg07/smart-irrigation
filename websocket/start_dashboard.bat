@echo off
title Smart Irrigation Dashboard
echo.
echo ===============================================
echo    Smart Irrigation System Dashboard
echo ===============================================
echo.

REM Check if Node.js is installed
node --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Node.js is not installed or not in PATH
    echo Please install Node.js from https://nodejs.org/
    pause
    exit /b 1
)

REM Check if we're in the correct directory
if not exist "server.js" (
    echo ERROR: server.js not found in current directory
    echo Please run this script from the websocket folder
    pause
    exit /b 1
)

REM Check if node_modules exists, if not install dependencies
if not exist "node_modules" (
    echo Installing dependencies...
    echo.
    npm install
    if %errorlevel% neq 0 (
        echo ERROR: Failed to install dependencies
        pause
        exit /b 1
    )
    echo.
    echo Dependencies installed successfully!
    echo.
)

REM Start the server
echo Starting Smart Irrigation Dashboard...
echo.
echo Dashboard will be available at:
echo - Local: http://localhost:3000
echo - Network: http://192.168.237.153:3000
echo.
echo Press Ctrl+C to stop the server
echo.

REM Start the server in background and open browser
echo Opening dashboard in browser...
start http://192.168.237.153:3000

node server.js

pause
