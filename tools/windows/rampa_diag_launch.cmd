@echo off
setlocal
rem =====================================================================
rem  Rampa - launcher behind the "Rampa diagnostika" desktop shortcut
rem
rem  Installed by rampa_setup.cmd into %LOCALAPPDATA%\Rampa. On every start:
rem    1. updates <Desktop>\Rampa to the newest origin/main (local edits go
rem       to "git stash", local commits to a backup branch - nothing is lost);
rem    2. creates/updates a private Python venv (%LOCALAPPDATA%\Rampa\venv)
rem       with PyQt6 and pyserial;
rem    3. opens gui\rampa_diag.py (standalone USB diagnostics for Rampa PCB1).
rem  Without internet it skips 1 and pip upgrade and runs offline.
rem  ASCII only on purpose (safe across all Windows code pages).
rem =====================================================================

title Rampa - diagnostika
set "APPDIR=%LOCALAPPDATA%\Rampa"
set "CFG=%APPDIR%\repo.txt"
set "VENV=%APPDIR%\venv"
set "VPY=%VENV%\Scripts\python.exe"
set "VPYW=%VENV%\Scripts\pythonw.exe"

call :find_git
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

rem ---- 1. Update repository from GitHub (if online) ---------------------
set "ONLINE=0"
echo ^>^> Aktualizuji %REPO%
git -C "%REPO%" fetch --prune origin
if errorlevel 1 (
    echo !! Nelze stahnout ^(offline?^) - pouzivam to, co je na disku.
) else (
    set "ONLINE=1"
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
)

if exist "%REPO%\tools\windows\rampa.ico" copy /y "%REPO%\tools\windows\rampa.ico" "%APPDIR%\rampa.ico" >nul 2>&1
if exist "%REPO%\tools\windows\rampa_launch.cmd" copy /y "%REPO%\tools\windows\rampa_launch.cmd" "%APPDIR%\rampa_launch.cmd" >nul 2>&1

rem ---- 2. Private Python venv and packages ------------------------------
set "VENV_OK=0"
if exist "%VPY%" (
    "%VPY%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
    if not errorlevel 1 set "VENV_OK=1"
)

if "%VENV_OK%"=="0" (
    call :find_python
    if not defined PY_EXE (
        echo CHYBA: Python 3.12 nenalezen - spustte znovu rampa_setup.cmd nebo nainstalujte Python 3.12.
        pause
        exit /b 1
    )
    echo ^>^> Vytvarim prostredi Pythonu v %VENV% ...
    "%PY_EXE%" -m venv --clear "%VENV%"
    if errorlevel 1 (
        echo CHYBA: python -m venv selhal.
        pause
        exit /b 1
    )
)

set "NEED_PIP=0"
"%VPY%" -c "import PyQt6, serial" >nul 2>&1
if errorlevel 1 set "NEED_PIP=1"

if "%ONLINE%"=="1" (
    if "%NEED_PIP%"=="1" (
        echo ^>^> Instaluji PyQt6 a pyserial ^(prvni spusteni chvili trva^)...
    ) else (
        echo ^>^> Kontroluji balicky Pythonu ^(pip^)...
    )
    "%VPY%" -m pip install --disable-pip-version-check -q -r "%REPO%\gui\requirements.txt"
    if errorlevel 1 (
        echo !! Nektere balicky se nepodarilo aktualizovat.
    )
) else (
    if "%NEED_PIP%"=="1" (
        echo CHYBA: V prostredi chybi PyQt6 / pyserial a nejde je stahnout ^(offline?^).
        pause
        exit /b 1
    )
)

"%VPY%" -c "import PyQt6, serial" >nul 2>&1
if errorlevel 1 (
    echo CHYBA: PyQt6 nebo pyserial se nepodarilo nacist.
    pause
    exit /b 1
)

rem ---- 3. Launch Rampa diagnostic GUI -----------------------------------
set "GUI=%REPO%\gui\rampa_diag.py"
if not exist "%GUI%" (
    echo CHYBA: Nenalezeno %GUI%
    pause
    exit /b 1
)

rem Quick import check so any startup error is shown clearly before detaching pythonw
"%VPY%" -c "import sys; sys.path.insert(0, r'%REPO%\gui'); import rampa_diag"
if errorlevel 1 (
    echo.
    echo CHYBA: Diagnostiku se nepodarilo nacist.
    pause
    exit /b 1
)

set "DATA=%USERPROFILE%\Documents\Rampa"
if not exist "%DATA%" mkdir "%DATA%" >nul 2>&1

echo ^>^> Spoustim Rampa diagnostiku...
if exist "%VPYW%" (
    start "" /D "%DATA%" "%VPYW%" "%GUI%"
) else (
    start "" /D "%DATA%" "%VPY%" "%GUI%"
)

timeout /t 2 /nobreak >nul
if exist "%REPO%\tools\windows\rampa_diag_launch.cmd" (
    copy /y "%REPO%\tools\windows\rampa_diag_launch.cmd" "%APPDIR%\rampa_diag_launch.cmd" >nul 2>&1 & exit /b 0
)
exit /b 0

:find_git
where git >nul 2>&1
if not errorlevel 1 exit /b 0
if exist "%ProgramFiles%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles%\Git\cmd"
if exist "%ProgramFiles(x86)%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles(x86)%\Git\cmd"
if exist "%LocalAppData%\Programs\Git\cmd\git.exe" set "PATH=%PATH%;%LocalAppData%\Programs\Git\cmd"
exit /b 0

:find_python
set "PY_EXE="
for /f "delims=" %%P in ('py -3.12 -c "import sys; print(sys.executable)" 2^>nul') do (
    if exist "%%P" set "PY_EXE=%%P"
)
if defined PY_EXE exit /b 0
if exist "%LocalAppData%\Programs\Python\Python312\python.exe" (
    set "PY_EXE=%LocalAppData%\Programs\Python\Python312\python.exe"
    exit /b 0
)
if exist "%ProgramFiles%\Python312\python.exe" (
    set "PY_EXE=%ProgramFiles%\Python312\python.exe"
    exit /b 0
)
for /f "delims=" %%P in ('py -3 -c "import sys; print(sys.executable if sys.version_info >= (3,10) else '')" 2^>nul') do (
    if exist "%%P" set "PY_EXE=%%P"
)
if defined PY_EXE exit /b 0
for /f "delims=" %%P in ('where python 2^>nul') do (
    if not defined PY_EXE (
        "%%P" -c "import sys; raise SystemExit(0 if sys.version_info >= (3,10) else 1)" >nul 2>&1
        if not errorlevel 1 set "PY_EXE=%%P"
    )
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
