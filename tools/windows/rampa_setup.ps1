# =====================================================================
#  Rampa - one-time setup on Windows
#
#  Run from cmd (one line):
#    powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://raw.githubusercontent.com/Tomasraketak/RampaCode/main/tools/windows/rampa_setup.ps1 | iex"
#
#  What it does (safe to run again at any time):
#    1. installs Git with winget, if it is missing;
#    2. puts the repository into  <Desktop>\Rampa  as a git clone. An
#       existing folder that is NOT a git clone is kept, renamed to
#       "Rampa (stara kopie <date>)";
#    3. creates the "Rampa firmware" shortcut on the desktop with the rampa
#       icon. The shortcut runs rampa_launch.ps1, which on every start
#       updates the repository and opens Rampa_MCU.ino (Arduino IDE).
#
#  ASCII only on purpose: Windows PowerShell 5.1 misreads UTF-8 without BOM.
# =====================================================================
# Another folder: set RAMPA_TARGET first, e.g.  set RAMPA_TARGET=D:\Rampa
$ErrorActionPreference = "Stop"
$Target = $env:RAMPA_TARGET
$RepoUrl = "https://github.com/Tomasraketak/RampaCode.git"
$AppDir  = Join-Path $env:LOCALAPPDATA "Rampa"

function Say($msg) { Write-Host ">> $msg" -ForegroundColor Cyan }
function Have($cmd) { return [bool](Get-Command $cmd -ErrorAction SilentlyContinue) }

$od = Join-Path $env:USERPROFILE "OneDrive\Desktop"
$desk = if (Test-Path $od) { $od } else { [Environment]::GetFolderPath("Desktop") }
if (-not $Target) { $Target = Join-Path $desk "Rampa" }
New-Item -ItemType Directory -Force -Path $AppDir | Out-Null

if (-not (Have "git")) {
    if (-not (Have "winget")) { throw "winget neni k dispozici. Nainstalujte 'App Installer' z Microsoft Store, nebo rucne Git." }
    Say "Instaluji Git..."
    winget install -e --id Git.Git --silent --accept-package-agreements --accept-source-agreements
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [Environment]::GetEnvironmentVariable("Path", "User")
    if (-not (Have "git")) { $env:Path += ";$env:ProgramFiles\Git\cmd" }
}

if (Test-Path (Join-Path $Target ".git")) {
    Say "$Target je uz git klon."
} else {
    if ((Test-Path $Target) -and (Get-ChildItem -Force $Target | Select-Object -First 1)) {
        $bak = "$Target (stara kopie $(Get-Date -Format 'yyyyMMdd-HHmm'))"
        Say "Slozka $Target neni git klon - prejmenovavam ji na '$bak' (nic se nemaze)."
        Rename-Item -Path $Target -NewName (Split-Path $bak -Leaf)
    }
    Say "Stahuji $RepoUrl -> $Target"
    git clone $RepoUrl $Target
    if ($LASTEXITCODE -ne 0) { throw "git clone $RepoUrl selhal." }
}

@{ repo = $Target } | ConvertTo-Json | Set-Content -Encoding ASCII (Join-Path $AppDir "config.json")
$win = Join-Path $Target "tools\windows"
Copy-Item -Force (Join-Path $win "rampa_launch.ps1") (Join-Path $AppDir "rampa_launch.ps1")
Copy-Item -Force (Join-Path $win "rampa.ico")        (Join-Path $AppDir "rampa.ico")

$desktop = [Environment]::GetFolderPath("Desktop")
$lnk = Join-Path $desktop "Rampa firmware.lnk"
$sh = New-Object -ComObject WScript.Shell
$s = $sh.CreateShortcut($lnk)
$s.TargetPath       = Join-Path $env:SystemRoot "System32\WindowsPowerShell\v1.0\powershell.exe"
$s.Arguments        = "-NoProfile -ExecutionPolicy Bypass -File `"$(Join-Path $AppDir 'rampa_launch.ps1')`""
$s.WorkingDirectory = $Target
$s.IconLocation     = (Join-Path $AppDir "rampa.ico") + ",0"
$s.Description      = "Rampa: aktualizuje kod a otevre firmware rampy"
$s.Save()
Say "Zastupce: $lnk"
& (Join-Path $AppDir "rampa_launch.ps1")
