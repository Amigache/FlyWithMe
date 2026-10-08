# Carga la configuración del banco (tools/bench.example.json + tools/bench.local.json).
# Uso desde otro script de tools/:  . (Join-Path $PSScriptRoot 'bench_config.ps1'); $cfg = Get-BenchConfig

function Get-BenchConfig {
    $toolsDir = $PSScriptRoot
    $examplePath = Join-Path $toolsDir 'bench.example.json'
    $localPath = Join-Path $toolsDir 'bench.local.json'

    $cfg = Get-Content -LiteralPath $examplePath -Raw -Encoding UTF8 | ConvertFrom-Json
    if (Test-Path -LiteralPath $localPath) {
        $overrides = Get-Content -LiteralPath $localPath -Raw -Encoding UTF8 | ConvertFrom-Json
        foreach ($prop in $overrides.PSObject.Properties) {
            $cfg | Add-Member -NotePropertyName $prop.Name -NotePropertyValue $prop.Value -Force
        }
    }
    else {
        Write-Warning "No existe tools/bench.local.json; se usan valores de ejemplo. Copia bench.example.json y ajústalo."
    }
    return $cfg
}

function Resolve-BenchPath([string]$path) {
    $repo = Split-Path -Parent $PSScriptRoot
    if ([IO.Path]::IsPathRooted($path)) { return $path }
    return Join-Path $repo $path
}
