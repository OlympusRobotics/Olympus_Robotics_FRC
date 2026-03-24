@echo off
REM fetch_logs.bat — Download and optionally purge logs from the roboRIO (Windows)
REM
REM Usage:
REM   tools\fetch_logs.bat              — fetch new logs
REM   tools\fetch_logs.bat --purge      — fetch then delete fetched logs from rio
REM   tools\fetch_logs.bat --purge-all  — fetch then delete ALL logs from rio

setlocal enabledelayedexpansion

set ROBOT_IP=10.49.82.2
set ROBOT_USER=lvuser
set REMOTE_LOG_DIR=/home/lvuser/logs
set SCRIPT_DIR=%~dp0
set LOCAL_LOG_DIR=%SCRIPT_DIR%..\logs
set SSH_OPTS=-o ConnectTimeout=5 -o StrictHostKeyChecking=no

set PURGE=0
set PURGE_ALL=0

if "%~1"=="--purge"     set PURGE=1
if "%~1"=="--purge-all" set PURGE_ALL=1
if "%~1"=="-h"          goto :usage
if "%~1"=="--help"      goto :usage

if not exist "%LOCAL_LOG_DIR%" mkdir "%LOCAL_LOG_DIR%"

echo Connecting to roboRIO at %ROBOT_IP%...
ssh %SSH_OPTS% %ROBOT_USER%@%ROBOT_IP% "echo ok" >nul 2>&1
if errorlevel 1 (
    echo ERROR: Cannot reach roboRIO at %ROBOT_IP%
    exit /b 1
)

echo Scanning remote logs...
set TMPFILE=%TEMP%\rio_logs_%RANDOM%.txt
ssh %SSH_OPTS% %ROBOT_USER%@%ROBOT_IP% "find %REMOTE_LOG_DIR% -maxdepth 2 -type f ( -name '*.wpilog' -o -name '*.hoot' )" > "%TMPFILE%" 2>nul

set FETCHED=0
set SKIPPED=0
set FETCHED_LIST=%TEMP%\rio_fetched_%RANDOM%.txt
if exist "%FETCHED_LIST%" del "%FETCHED_LIST%"

for /f "usebackq delims=" %%F in ("%TMPFILE%") do (
    for %%N in (%%F) do set FILENAME=%%~nxN
    if exist "%LOCAL_LOG_DIR%\!FILENAME!" (
        set /a SKIPPED+=1
    ) else (
        echo   Downloading !FILENAME!...
        scp %SSH_OPTS% %ROBOT_USER%@%ROBOT_IP%:"%%F" "%LOCAL_LOG_DIR%\" >nul 2>&1
        set /a FETCHED+=1
        echo %%F>> "%FETCHED_LIST%"
    )
)

echo.
echo Downloaded: !FETCHED! file(s), skipped !SKIPPED! already-local file(s)

if %PURGE_ALL%==1 (
    echo.
    echo Purging ALL logs from roboRIO...
    ssh %SSH_OPTS% %ROBOT_USER%@%ROBOT_IP% "find %REMOTE_LOG_DIR% -maxdepth 2 -type f ( -name '*.wpilog' -o -name '*.hoot' -o -name '*.revlog' ) -exec rm -f {} ; && find %REMOTE_LOG_DIR% -mindepth 1 -maxdepth 1 -type d -exec rmdir {} ; 2>/dev/null"
    echo Done — roboRIO logs purged.
) else if %PURGE%==1 if exist "%FETCHED_LIST%" (
    echo.
    echo Purging !FETCHED! fetched log(s) from roboRIO...
    for /f "usebackq delims=" %%F in ("%FETCHED_LIST%") do (
        ssh %SSH_OPTS% %ROBOT_USER%@%ROBOT_IP% "rm -f '%%F'" >nul 2>&1
    )
    ssh %SSH_OPTS% %ROBOT_USER%@%ROBOT_IP% "find %REMOTE_LOG_DIR% -mindepth 1 -maxdepth 1 -type d -exec rmdir {} ; 2>/dev/null" >nul 2>&1
    echo Done.
)

REM Cleanup temp files
if exist "%TMPFILE%" del "%TMPFILE%"
if exist "%FETCHED_LIST%" del "%FETCHED_LIST%"

echo.
echo Local logs: %LOCAL_LOG_DIR%
dir "%LOCAL_LOG_DIR%\*.wpilog" "%LOCAL_LOG_DIR%\*.hoot" 2>nul
goto :eof

:usage
echo Usage: %~nx0 [--purge ^| --purge-all]
echo   --purge      Delete fetched logs from the roboRIO after download
echo   --purge-all  Delete ALL logs from the roboRIO after download
exit /b 0
