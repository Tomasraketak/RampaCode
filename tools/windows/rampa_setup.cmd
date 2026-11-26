@echo off
setlocal
rem =====================================================================
rem  Rampa - one-time setup on Windows (pure cmd, no PowerShell needed)
rem
rem  Run from cmd (one line):
rem    curl.exe -fsSL https://raw.githubusercontent.com/Tomasraketak/RampaCode/main/tools/windows/rampa_setup.cmd -o "%TEMP%\rampa_setup.cmd" && "%TEMP%\rampa_setup.cmd"
rem
rem  What it does (safe to run again at any time):
rem    1. installs Git with winget, if it is missing;
rem    2. puts the repository into  <Desktop>\Rampa  as a git clone. An
rem       existing folder that is NOT a git clone is kept, renamed to
rem       "Rampa (stara kopie <timestamp>)";
rem    3. creates the "Rampa firmware" shortcut on the desktop with the rampa
rem       icon. The shortcut runs rampa_launch.cmd, which on every start
rem       updates the repository and opens Rampa_MCU.ino (Arduino IDE).
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
rem --- Save config and copy launcher files ---
>"%APPDIR%\repo.txt" echo %TARGET%
set "WINDIR_SRC=%TARGET%\tools\windows"
copy /y "%WINDIR_SRC%\rampa_launch.cmd" "%APPDIR%\rampa_launch.cmd" >nul
if errorlevel 1 (
    echo CHYBA: Nepodarilo se zkopirovat rampa_launch.cmd do %APPDIR%.
    pause
    exit /b 1
)
if exist "%WINDIR_SRC%\rampa.ico" copy /y "%WINDIR_SRC%\rampa.ico" "%APPDIR%\rampa.ico" >nul

rem --- Create Desktop shortcut "Rampa firmware.lnk" ---
set "LNK=%DESKTOP%\Rampa firmware.lnk"
set "VBS=%TEMP%\rampa_mklnk.vbs"
>"%VBS%" echo Set sh = CreateObject("WScript.Shell")
>>"%VBS%" echo Set s = sh.CreateShortcut(WScript.Arguments(0))
>>"%VBS%" echo s.TargetPath = WScript.Arguments(1)
>>"%VBS%" echo s.WorkingDirectory = WScript.Arguments(2)
>>"%VBS%" echo s.IconLocation = WScript.Arguments(3)
>>"%VBS%" echo s.Description = "Rampa: aktualizuje kod a otevre firmware rampy"
>>"%VBS%" echo s.Save
cscript //nologo "%VBS%" "%LNK%" "%APPDIR%\rampa_launch.cmd" "%TARGET%" "%APPDIR%\rampa.ico,0" >nul 2>&1
del /q "%VBS%" >nul 2>&1

if exist "%LNK%" (
    echo ^>^> Zastupce: %LNK%
) else (
    copy /y "%APPDIR%\rampa_launch.cmd" "%DESKTOP%\Rampa firmware.cmd" >nul 2>&1
    echo ^>^> Zastupce: %DESKTOP%\Rampa firmware.cmd
)

call "%APPDIR%\rampa_launch.cmd"
exit /b %ERRORLEVEL%

:find_git
where git >nul 2>&1
if not errorlevel 1 exit /b 0
if exist "%ProgramFiles%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles%\Git\cmd"
if exist "%ProgramFiles(x86)%\Git\cmd\git.exe" set "PATH=%PATH%;%ProgramFiles(x86)%\Git\cmd"
if exist "%LocalAppData%\Programs\Git\cmd\git.exe" set "PATH=%PATH%;%LocalAppData%\Programs\Git\cmd"
where git >nul 2>&1
exit /b %ERRORLEVEL%

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
