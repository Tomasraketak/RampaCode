@echo off
setlocal
rem =====================================================================
rem  Rampa - launcher behind the "Rampa firmware" desktop shortcut
rem
rem  Installed by rampa_setup.cmd into %LOCALAPPDATA%\Rampa. On every start:
rem    1. updates <Desktop>\Rampa to the newest origin/main (local edits go
rem       to "git stash", local commits to a backup branch - nothing is lost);
rem    2. opens Rampa_MCU\Rampa_MCU.ino with its default program (Arduino IDE)
rem       or, if none is associated, the folder in Explorer.
rem  Without internet it skips 1.
rem  ASCII only on purpose (safe across all Windows code pages).
rem =====================================================================

title Rampa - aktualizace
set "APPDIR=%LOCALAPPDATA%\Rampa"
set "CFG=%APPDIR%\repo.txt"

where git >nul 2>&1
if errorlevel 1 (
    if exist "%ProgramFiles%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles%\Git\cmd"
    if exist "%ProgramFiles(x86)%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles(x86)%\Git\cmd"
    if exist "%LocalAppData%\Programs\Git\cmd\git.exe" set "PATH=%PATH%;%LocalAppData%\Programs\Git\cmd"
)

if not exist "%CFG%" (
    echo CHYBA: Chybi %CFG% - spustte nejdriv rampa_setup.cmd.
    pause
    exit /b 1
)

set "REPO="
set /p REPO=<"%CFG%"
if not exist "%REPO%\.git" (
    echo CHYBA: "%REPO%" neni git klon - spustte znovu rampa_setup.cmd.
    pause
    exit /b 1
)

echo ^>^> Aktualizuji %REPO%
git -C "%REPO%" fetch --prune origin
if errorlevel 1 (
    echo !! Nelze stahnout ^(offline?^) - pouzivam to, co je na disku.
    goto :open_ino
)

set "DIRTY="
for /f "delims=" %%S in ('git -C "%REPO%" status --porcelain --untracked-files=no 2^>nul') do set "DIRTY=1"
if defined DIRTY (
    call :get_stamp
    git -C "%REPO%" stash push -m "auto-zaloha pred aktualizaci %STAMP%" >nul
    echo !! Upravene soubory byly odlozeny do 'git stash'. Zpet: git stash pop
)

set "BR="
for /f "delims=" %%B in ('git -C "%REPO%" rev-parse --abbrev-ref HEAD 2^>nul') do set "BR=%%B"
if /i not "%BR%"=="main" (
    git -C "%REPO%" checkout -q main 2>nul
    if errorlevel 1 git -C "%REPO%" checkout -q -B main origin/main
)

git -C "%REPO%" merge --ff-only -q origin/main 2>nul
if errorlevel 1 (
    call :get_stamp
    git -C "%REPO%" branch "zaloha-%STAMP%" >nul 2>&1
    git -C "%REPO%" reset -q --hard origin/main
    echo !! Lokalni commity odlozeny do vetve 'zaloha-%STAMP%', slozka je ted presne origin/main.
)

git -C "%REPO%" log -1 --format=">>    %%h %%ad %%s" --date=short

:open_ino
if exist "%REPO%\tools\windows\rampa.ico" copy /y "%REPO%\tools\windows\rampa.ico" "%APPDIR%\rampa.ico" >nul 2>&1

set "INO=%REPO%\Rampa_MCU\Rampa_MCU.ino"
echo ^>^> Otviram %INO%
start "" "%INO%" 2>nul
if errorlevel 1 (
    echo !! K .ino neni prirazen program ^(Arduino IDE^) - otviram slozku.
    explorer "%REPO%\Rampa_MCU"
)

timeout /t 2 /nobreak >nul

if exist "%REPO%\tools\windows\rampa_launch.cmd" (
    copy /y "%REPO%\tools\windows\rampa_launch.cmd" "%APPDIR%\rampa_launch.cmd" >nul 2>&1 & exit /b 0
)
exit /b 0

:get_stamp
setlocal EnableDelayedExpansion
set "RAW=%DATE%_%TIME:~0,5%"
set "OUT="
:stamp_loop
if not defined RAW goto :stamp_done
set "CH=!RAW:~0,1!"
set "RAW=!RAW:~1!"
for %%D in (0 1 2 3 4 5 6 7 8 9 _) do if "!CH!"=="%%D" set "OUT=!OUT!!CH!"
goto :stamp_loop
:stamp_done
endlocal & set "STAMP=%OUT%"
exit /b 0
