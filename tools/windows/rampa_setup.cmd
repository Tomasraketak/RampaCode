@echo off
setlocal
rem =====================================================================
rem  Rampa - one-time setup on Windows (pure cmd, no PowerShell needed)
rem
rem  Run from cmd (one line):
rem    curl.exe -fsSL https://raw.githubusercontent.com/Tomasraketak/RampaCode/main/tools/windows/rampa_setup.cmd -o "%TEMP%\rampa_setup.cmd" && "%TEMP%\rampa_setup.cmd"
rem
rem  What it does (safe to run again at any time):
rem    1. installs Git and Python 3.12 with winget, if missing;
rem    2. puts the repository into  <Desktop>\Rampa  as a git clone. An
rem       existing folder that is NOT a git clone is kept, renamed to
rem       "Rampa (stara kopie <timestamp>)";
rem    3. creates two desktop shortcuts with the rampa icon:
rem       - "Rampa diagnostika" -> updates repo & opens gui\rampa_diag.py
rem       - "Rampa firmware"    -> updates repo & opens Rampa_MCU.ino
rem
rem  Another folder: set RAMPA_TARGET first, e.g.  set RAMPA_TARGET=D:\Rampa
rem  ASCII only on purpose (safe across all Windows code pages).
rem =====================================================================

set "REPO_URL=https://github.com/Tomasraketak/RampaCode.git"
set "APPDIR=%LOCALAPPDATA%\Rampa"

rem --- Locate Desktop (respects OneDrive and localized Czech "Plocha") ---
set "DESKTOP="
for /f "tokens=1,2,*" %%A in ('reg query "HKCU\Software\Microsoft\Windows\CurrentVersion\Explorer\User Shell Folders" /v Desktop 2^>nul') do (
    if /i "%%A"=="Desktop" call set "DESKTOP=%%C"
)
if not defined DESKTOP set "DESKTOP=%USERPROFILE%\Desktop"
if not exist "%DESKTOP%" (
    if exist "%USERPROFILE%\OneDrive\Desktop" (
        set "DESKTOP=%USERPROFILE%\OneDrive\Desktop"
    ) else (
        set "DESKTOP=%USERPROFILE%\Desktop"
    )
)

if defined RAMPA_TARGET (
    set "TARGET=%RAMPA_TARGET%"
) else (
    set "TARGET=%DESKTOP%\Rampa"
)

if not exist "%APPDIR%" mkdir "%APPDIR%"

rem --- Ensure Git is available ---
call :find_git
if errorlevel 1 (
    where winget >nul 2>&1
    if errorlevel 1 (
        echo CHYBA: Git ani winget nejsou k dispozici. Nainstalujte Git z https://git-scm.com/download/win a spustte znovu.
        pause
        exit /b 1
    )
    echo ^>^> Instaluji Git...
    winget install -e --id Git.Git --silent --accept-package-agreements --accept-source-agreements
    call :find_git
    if errorlevel 1 (
        echo CHYBA: Git byl nainstalovan, ale neni v PATH. Zavrete cmd, otevrete nove okno a spustte prikaz znovu.
        pause
        exit /b 1
    )
)

rem --- Ensure Python 3.12 (or >= 3.10) is available ---
call :find_python
if not defined PY_EXE (
    where winget >nul 2>&1
    if errorlevel 1 (
        echo CHYBA: Python 3.12 ani winget nejsou k dispozici. Nainstalujte Python 3.12 z https://www.python.org/downloads/ a spustte znovu.
        pause
        exit /b 1
    )
    echo ^>^> Instaluji Python 3.12...
    winget install -e --id Python.Python.3.12 --scope user --silent --accept-package-agreements --accept-source-agreements
    call :find_python
    if not defined PY_EXE (
        echo CHYBA: Python 3.12 se nepodarilo najit po instalaci. Zavrete cmd, otevrete nove okno a spustte prikaz znovu.
        pause
        exit /b 1
    )
)
echo ^>^> Python: %PY_EXE%

rem --- Clone repository (or keep existing clone / back up non-git folder) ---
if exist "%TARGET%\.git" (
    echo ^>^> %TARGET% je uz git klon.
    goto :after_clone
)

if not exist "%TARGET%" goto :do_clone

set "HAS_FILES="
for /f "delims=" %%F in ('dir /b /a "%TARGET%" 2^>nul') do set "HAS_FILES=1"
if not defined HAS_FILES goto :do_clone

call :get_stamp
for %%I in ("%TARGET%") do set "BASE_NAME=%%~nxI"
set "BAK_NAME=%BASE_NAME% (stara kopie %STAMP%)"
echo ^>^> Slozka %TARGET% neni git klon - prejmenovavam ji na "%BAK_NAME%" ^(nic se nemaze^).
ren "%TARGET%" "%BAK_NAME%"
if errorlevel 1 (
    echo CHYBA: Nepodarilo se prejmenovat "%TARGET%". Zavrete programy, ktere ji pouzivaji, a zkuste to znovu.
    pause
    exit /b 1
)

:do_clone
echo ^>^> Stahuji %REPO_URL% -^> %TARGET%
git clone "%REPO_URL%" "%TARGET%"
if errorlevel 1 (
    echo CHYBA: git clone %REPO_URL% selhal.
    pause
    exit /b 1
)

:after_clone
rem --- Refresh repo if already cloned so latest launchers exist ---
git -C "%TARGET%" fetch --prune origin >nul 2>&1
git -C "%TARGET%" merge --ff-only -q origin/main >nul 2>&1

rem --- Save config and copy launcher files ---
>"%APPDIR%\repo.txt" echo %TARGET%
set "WINDIR_SRC=%TARGET%\tools\windows"
copy /y "%WINDIR_SRC%\rampa_diag_launch.cmd" "%APPDIR%\rampa_diag_launch.cmd" >nul
if errorlevel 1 (
    echo CHYBA: Nepodarilo se zkopirovat rampa_diag_launch.cmd do %APPDIR%.
    pause
    exit /b 1
)
copy /y "%WINDIR_SRC%\rampa_launch.cmd" "%APPDIR%\rampa_launch.cmd" >nul
if exist "%WINDIR_SRC%\rampa.ico" copy /y "%WINDIR_SRC%\rampa.ico" "%APPDIR%\rampa.ico" >nul

rem --- Create Desktop shortcuts ("Rampa diagnostika" and "Rampa firmware") ---
call :make_shortcut "%DESKTOP%\Rampa diagnostika.lnk" "%APPDIR%\rampa_diag_launch.cmd" "%TARGET%" "%APPDIR%\rampa.ico,0" "Rampa: aktualizuje kod a spusti USB diagnostiku rampy" "%DESKTOP%\Rampa diagnostika.cmd"
call :make_shortcut "%DESKTOP%\Rampa firmware.lnk" "%APPDIR%\rampa_launch.cmd" "%TARGET%" "%APPDIR%\rampa.ico,0" "Rampa: aktualizuje kod a otevre firmware rampy (Arduino IDE)" "%DESKTOP%\Rampa firmware.cmd"

echo ^>^> Prvni spusteni diagnostiky ^(vytvori prostredi Pythonu, to chvili trva^)...
call "%APPDIR%\rampa_diag_launch.cmd"
exit /b %ERRORLEVEL%

:make_shortcut
set "LNK=%~1"
set "TGT=%~2"
set "WRK=%~3"
set "ICO=%~4"
set "DSC=%~5"
set "FBK=%~6"
set "VBS=%TEMP%\rampa_mklnk.vbs"
>"%VBS%" echo Set sh = CreateObject("WScript.Shell")
>>"%VBS%" echo Set s = sh.CreateShortcut(WScript.Arguments(0))
>>"%VBS%" echo s.TargetPath = WScript.Arguments(1)
>>"%VBS%" echo s.WorkingDirectory = WScript.Arguments(2)
>>"%VBS%" echo s.IconLocation = WScript.Arguments(3)
>>"%VBS%" echo s.Description = WScript.Arguments(4)
>>"%VBS%" echo s.Save
cscript //nologo "%VBS%" "%LNK%" "%TGT%" "%WRK%" "%ICO%" "%DSC%" >nul 2>&1
del /q "%VBS%" >nul 2>&1
if exist "%LNK%" (
    echo ^>^> Zastupce: %LNK%
) else (
    copy /y "%TGT%" "%FBK%" >nul 2>&1
    echo ^>^> Zastupce: %FBK%
)
exit /b 0

:find_git
where git >nul 2>&1
if not errorlevel 1 exit /b 0
if exist "%ProgramFiles%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles%\Git\cmd"
if exist "%ProgramFiles(x86)%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles(x86)%\Git\cmd"
if exist "%LocalAppData%\Programs\Git\cmd\git.exe" set "PATH=%PATH%;%LocalAppData%\Programs\Git\cmd"
where git >nul 2>&1
exit /b %ERRORLEVEL%

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
