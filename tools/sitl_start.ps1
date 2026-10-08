# Arranca el banco de pruebas SITL: 2 aviones ArduPlane + puentes a las placas.
# Valores (ruta de ArduPlane, puertos COM, coordenadas de inicio) en tools/bench.local.json.
# Uso:  powershell -ExecutionPolicy Bypass -File tools\sitl_start.ps1 [-MasterCom COMx] [-SlaveCom COMy]
param(
    [string]$Exe,
    [string]$MasterCom,
    [string]$SlaveCom,
    # yaw 180 => despegue hacia el SUR (evita las montanas que hay al norte)
    [string]$LeaderHome,
    [string]$FollowerHome,
    [string]$Python = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe",
    [int]$Baud = 57600
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot 'bench_config.ps1')
$cfg = Get-BenchConfig
if (-not $Exe) { $Exe = $cfg.sitl_exe }
if (-not $MasterCom) { $MasterCom = $cfg.leader_com }
if (-not $SlaveCom) { $SlaveCom = $cfg.slave_com }
if (-not $LeaderHome) { $LeaderHome = $cfg.leader_home }
if (-not $FollowerHome) { $FollowerHome = $cfg.follower_home }

$repo = Split-Path -Parent $PSScriptRoot
$base = Resolve-BenchPath $cfg.sitl_base
New-Item -ItemType Directory -Force "$base\l0", "$base\l1" | Out-Null

if (-not (Test-Path $Exe)) { Write-Error "No existe el binario SITL: $Exe (configura sitl_exe en tools/bench.local.json)" }
$leaderParm = Join-Path $repo "tools\sitl\leader.parm"
$followerParm = Join-Path $repo "tools\sitl\follower.parm"

Get-Process ArduPlane, ArduCopter, python -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 1

Write-Host "Lanzando SITL lider (sysid 1, TCP 5760)..." -ForegroundColor Cyan
Start-Process $Exe -ArgumentList @('--instance','0','--serial0','tcp:5760','--sysid','1','--model','plane',
    '--home',$LeaderHome,'--defaults',$leaderParm) -WorkingDirectory "$base\l0" `
    -RedirectStandardOutput "$base\l0\out.log" -RedirectStandardError "$base\l0\err.log"

Write-Host "Lanzando SITL seguidor (sysid 2, TCP 5770)..." -ForegroundColor Cyan
Start-Process $Exe -ArgumentList @('--instance','1','--serial0','tcp:5770','--sysid','2','--model','plane',
    '--home',$FollowerHome,'--defaults',$followerParm) -WorkingDirectory "$base\l1" `
    -RedirectStandardOutput "$base\l1\out.log" -RedirectStandardError "$base\l1\err.log"

Start-Sleep -Seconds 11
Write-Host "Arrancando puentes serie..." -ForegroundColor Cyan
Start-Process $Python -ArgumentList @("$repo\tools\sitl_bridge.py",'--tcp','127.0.0.1:5760','--port',$MasterCom,'--baud',$Baud,'--print-serial') `
    -WorkingDirectory $repo -RedirectStandardOutput "$base\b0.log" -RedirectStandardError "$base\b0.err"
Start-Process $Python -ArgumentList @("$repo\tools\sitl_bridge.py",'--tcp','127.0.0.1:5770','--port',$SlaveCom,'--baud',$Baud,'--print-serial') `
    -WorkingDirectory $repo -RedirectStandardOutput "$base\b1.log" -RedirectStandardError "$base\b1.err"

Write-Host ""
Write-Host "SITL + puentes arrancados." -ForegroundColor Green
Write-Host "Mission Planner (Connection TCP, host 127.0.0.1):"
Write-Host "   Lider    -> puerto 5762"
Write-Host "   Seguidor -> puerto 5772"
Write-Host ""
Write-Host "Para que el seguidor siga:"
Write-Host "   $Python tools\sitl_guided.py --conn tcp:127.0.0.1:5773 --mode 15"
Write-Host "Para despegar el lider:"
Write-Host "   $Python tools\sitl_takeoff.py --conn tcp:127.0.0.1:5763 --mode 13 --alt 120"
