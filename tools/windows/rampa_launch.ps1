# =====================================================================
#  Rampa - launcher behind the "Rampa firmware" desktop shortcut
#
#  Installed by rampa_setup.ps1 into %LOCALAPPDATA%\Rampa. On every start:
#    1. updates <Desktop>\Rampa to the newest origin/main (local edits go
#       to "git stash", local commits to a backup branch - nothing is lost);
#    2. no argument: opens Rampa_MCU\Rampa_MCU.ino with its default program
#       (Arduino IDE) or, if none is associated, the folder in Explorer;
#       argument "diag": keeps %LOCALAPPDATA%\Rampa\venv up to date and starts
#       gui\rampa_diag.py ("diag demo" = simulated rampa).
#  Without internet it skips 1.  Log: %LOCALAPPDATA%\Rampa\launch.log
#  ASCII only on purpose: Windows PowerShell 5.1 misreads UTF-8 without BOM.
# =====================================================================
param([string]$Mode = "", [string]$Opt = "")
$ErrorActionPreference = "Continue"
$AppDir = Join-Path $env:LOCALAPPDATA "Rampa"
try { Start-Transcript -Path (Join-Path $AppDir "launch.log") -Force | Out-Null } catch {}
$Host.UI.RawUI.WindowTitle = "Rampa - aktualizace"
function Say($msg)  { Write-Host ">> $msg" -ForegroundColor Cyan }
function Warn($msg) { Write-Host "!! $msg" -ForegroundColor Yellow }
function Fail($msg) { Write-Host "CHYBA: $msg" -ForegroundColor Red; Read-Host "Enter = zavrit"; exit 1 }

$env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [Environment]::GetEnvironmentVariable("Path", "User")
$cfgFile = Join-Path $AppDir "config.json"
if (-not (Test-Path $cfgFile)) { Fail "Chybi $cfgFile - spustte nejdriv rampa_setup.ps1." }
$Repo = (Get-Content $cfgFile -Raw | ConvertFrom-Json).repo
if (-not (Test-Path (Join-Path $Repo ".git"))) { Fail "$Repo neni git klon - spustte znovu rampa_setup.ps1." }

Say "Aktualizuji $Repo"
git -C $Repo fetch --prune origin
if ($LASTEXITCODE -ne 0) {
    Warn "Nelze stahnout (offline?) - pouzivam to, co je na disku."
} else {
    if (git -C $Repo status --porcelain --untracked-files=no) {
        $msg = "auto-zaloha pred aktualizaci $(Get-Date -Format 'yyyy-MM-dd HH:mm')"
        git -C $Repo stash push -m $msg | Out-Null
        Warn "Upravene soubory byly odlozeny do 'git stash' ($msg). Zpet: git stash pop"
    }
    $br = (git -C $Repo rev-parse --abbrev-ref HEAD).Trim()
    if ($br -ne "main") { git -C $Repo checkout -q main 2>$null; if ($LASTEXITCODE -ne 0) { git -C $Repo checkout -q -B main origin/main } }
    git -C $Repo merge --ff-only -q origin/main
    if ($LASTEXITCODE -ne 0) {
        $bak = "zaloha-$(Get-Date -Format 'yyyyMMdd-HHmm')"
        git -C $Repo branch $bak | Out-Null
        git -C $Repo reset -q --hard origin/main
        Warn "Lokalni commity odlozeny do vetve '$bak', slozka je ted presne origin/main."
    }
    Say ("   " + (git -C $Repo log -1 --format="%h %ad %s" --date=short))
}
foreach ($f in "rampa_launch.ps1", "rampa.ico") {
    $src = Join-Path $Repo "tools\windows\$f"
    if (Test-Path $src) { Copy-Item -Force $src (Join-Path $AppDir $f) -ErrorAction SilentlyContinue }
}
if ($Mode -eq "diag") {
    $Venv = Join-Path $AppDir "venv"
    $VPy  = Join-Path $Venv "Scripts\python.exe"
    $VPyW = Join-Path $Venv "Scripts\pythonw.exe"
    if (-not (Test-Path $VPy)) {
        $base = $null
        if (Get-Command py -ErrorAction SilentlyContinue) {
            $p = & py -3.12 -c "import sys; print(sys.executable)" 2>$null
            if ($LASTEXITCODE -eq 0 -and $p) { $base = $p.Trim() }
        }
        if (-not $base) { $c = Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"; if (Test-Path $c) { $base = $c } }
        if (-not $base) { Fail "Python 3.12 nenalezen - spustte znovu rampa_setup.ps1." }
        Say "Vytvarim prostredi Pythonu v $Venv"
        & $base -m venv $Venv
        if ($LASTEXITCODE -ne 0) { Fail "python -m venv selhal." }
    }
    Say "Aktualizuji balicky (pip)..."
    & $VPy -m pip install --upgrade --disable-pip-version-check -q -r (Join-Path $Repo "gui\requirements.txt")
    & $VPy -c "import PyQt6, serial" 2>$null
    if ($LASTEXITCODE -ne 0) { Fail "V prostredi chybi PyQt6 / pyserial a nejde je stahnout (offline?)." }
    $gui = Join-Path $Repo "gui\rampa_diag.py"
    $guiArgs = "`"$gui`""
    if ($Opt -eq "demo") { $guiArgs += " --demo" }
    $data = Join-Path ([Environment]::GetFolderPath("MyDocuments")) "Rampa"
    New-Item -ItemType Directory -Force -Path $data | Out-Null
    $errLog = Join-Path $AppDir "gui_error.log"
    Say "Spoustim diagnostiku rampy..."
    $p = Start-Process -FilePath $VPyW -ArgumentList $guiArgs -WorkingDirectory $data -RedirectStandardError $errLog -PassThru
    Start-Sleep -Seconds 6
    if ($p.HasExited -and $p.ExitCode -ne 0) {
        if (Test-Path $errLog) { Get-Content $errLog -Tail 40 | ForEach-Object { Write-Host $_ } }
        Fail "Diagnostika se nespustila (chyba vyse, ulozeno v $errLog)."
    }
    try { Stop-Transcript | Out-Null } catch {}
    exit 0
}

$ino = Join-Path $Repo "Rampa_MCU\Rampa_MCU.ino"
try { Invoke-Item $ino -ErrorAction Stop; Say "Otviram $ino" }
catch { Warn "K .ino neni prirazen program (Arduino IDE) - otviram slozku."; Invoke-Item (Join-Path $Repo "Rampa_MCU") }
try { Stop-Transcript | Out-Null } catch {}
Start-Sleep -Seconds 2
