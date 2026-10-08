param(
  [switch]$NoKill
)
# Reinicia el banco SITL (2 ArduPlane + puentes). Valores en tools/bench.local.json.
$ErrorActionPreference='SilentlyContinue'
. (Join-Path $PSScriptRoot 'bench_config.ps1')
$cfg = Get-BenchConfig
$repo = Split-Path -Parent $PSScriptRoot
$stable = $cfg.sitl_exe
$base = Resolve-BenchPath $cfg.sitl_base
New-Item -ItemType Directory -Force "$base\l0", "$base\l1" | Out-Null
$py="$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
$env:PYTHONIOENCODING='utf-8'
if(-not $NoKill){
  Get-Process ArduPlane,ArduCopter -ErrorAction SilentlyContinue | Stop-Process -Force
  Start-Sleep -Seconds 2
}
$lp=(Resolve-Path "$PSScriptRoot\sitl\leader.parm" -ErrorAction SilentlyContinue).Path
$fp=(Resolve-Path "$PSScriptRoot\sitl\follower.parm").Path
Start-Process $stable -ArgumentList @('--instance','0','--serial0','tcp:5760','--sysid','1','--model','plane','--home',$cfg.leader_home,'--defaults',$lp) -WorkingDirectory "$base\l0"
Start-Process $stable -ArgumentList @('--instance','1','--serial0','tcp:5770','--sysid','2','--model','plane','--home',$cfg.follower_home,'--defaults',$fp) -WorkingDirectory "$base\l1"
Start-Sleep -Seconds 12
Start-Process $py -ArgumentList @('.\tools\sitl_bridge.py','--tcp','127.0.0.1:5760','--port',$cfg.leader_com,'--baud','57600') -WorkingDirectory $repo
Start-Process $py -ArgumentList @('.\tools\sitl_bridge.py','--tcp','127.0.0.1:5770','--port',$cfg.slave_com,'--baud','57600') -WorkingDirectory $repo
Start-Sleep -Seconds 14
Write-Output "bench listo"
