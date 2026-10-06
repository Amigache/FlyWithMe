param(
  [switch]$NoKill
)
$ErrorActionPreference='SilentlyContinue'
$stable="$env:LOCALAPPDATA\Temp\opencode\sitl-stable\ArduPlane.exe"
$base="$env:LOCALAPPDATA\Temp\opencode\sitl"
$py="$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
$env:PYTHONIOENCODING='utf-8'
if(-not $NoKill){
  Get-Process ArduPlane,ArduCopter -ErrorAction SilentlyContinue | Stop-Process -Force
  Start-Sleep -Seconds 2
}
$lp=(Resolve-Path "$PSScriptRoot\..\tools\sitl\leader.parm" -ErrorAction SilentlyContinue).Path
if(-not $lp){ $lp=(Resolve-Path ".\tools\sitl\leader.parm").Path }
$fp=(Resolve-Path ".\tools\sitl\follower.parm").Path
Start-Process $stable -ArgumentList @('--instance','0','--serial0','tcp:5760','--sysid','1','--model','plane','--home','0.000000,0.000000,302,180','--defaults',$lp) -WorkingDirectory "$base\l0"
Start-Process $stable -ArgumentList @('--instance','1','--serial0','tcp:5770','--sysid','2','--model','plane','--home','0.000000,0.000000,302,180','--defaults',$fp) -WorkingDirectory "$base\l1"
Start-Sleep -Seconds 12
Start-Process $py -ArgumentList @('.\tools\sitl_bridge.py','--tcp','127.0.0.1:5760','--port','COMx','--baud','57600') -WorkingDirectory (Get-Location)
Start-Process $py -ArgumentList @('.\tools\sitl_bridge.py','--tcp','127.0.0.1:5770','--port','COMx','--baud','57600') -WorkingDirectory (Get-Location)
Start-Sleep -Seconds 14
Write-Output "bench listo"
