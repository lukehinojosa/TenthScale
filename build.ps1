# Builds CppTemplate on Windows after install.bat has run once. The README's Command line section lists the
# options.
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Config = 'Debug',
    [switch]$Web,
    [switch]$Run,
    [switch]$Sandbox,
    [switch]$SkipTests,
    [switch]$GpuCompute
)

$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

# Picks up tools installed after this window opened, so nothing needs restarting after install.bat
$env:Path = @([Environment]::GetEnvironmentVariable('Path', 'Machine'), [Environment]::GetEnvironmentVariable('Path', 'User'), $env:Path) -join ';'

# Native commands write progress to stderr, so only the exit code means failure
function Invoke-Step([string]$Name, [scriptblock]$Command)
{
    Write-Host ""
    Write-Host "== $Name" -ForegroundColor Cyan
    & $Command
    if ($LASTEXITCODE -ne 0)
    {
        throw "$Name failed (exit $LASTEXITCODE)"
    }
}

function Assert-Tool([string]$Name)
{
    if (-not (Get-Command $Name -ErrorAction SilentlyContinue))
    {
        throw "$Name was not found. Run install.bat first."
    }
}

# Loads the compiler environment from whichever Visual Studio version has the C++ tools
function Enter-Msvc
{
    if (Get-Command cl.exe -ErrorAction SilentlyContinue)
    {
        return
    }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere))
    {
        throw 'Visual Studio was not found. Run install.bat first.'
    }
    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1
    $vcvars = if ($vsPath) { Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat' } else { $null }
    if (-not $vcvars -or -not (Test-Path $vcvars))
    {
        throw 'The Visual Studio C++ tools were not found. Run install.bat first.'
    }
    foreach ($line in (& cmd.exe /c "`"$vcvars`" >nul && set"))
    {
        $name, $value = $line -split '=', 2
        if ($value)
        {
            Set-Item -Path "env:$name" -Value $value
        }
    }
}

$extraArgs = @()
if ($GpuCompute)
{
    $extraArgs += '-DTEMPLATE_GPU_COMPUTE=ON'
}

Assert-Tool 'cmake'
Assert-Tool 'ninja'

if ($Web)
{
    # cmake\EmscriptenToolchain.cmake finds emsdk, so emsdk_env isn't needed
    Invoke-Step 'Configure (web)' { cmake --preset web @extraArgs }
    Invoke-Step 'Build (web)' { cmake --build --preset web }
    Write-Host ""
    Write-Host "Done. The page is in build\web\bin." -ForegroundColor Green
    if ($Run)
    {
        Assert-Tool 'python'
        Write-Host "Serving on http://localhost:8080 (Ctrl+C or close the window to stop)"
        Start-Process ($(if ($Sandbox) { 'http://localhost:8080/sandbox.html' } else { 'http://localhost:8080' }))
        & python -m http.server 8080 --directory build\web\bin
    }
    exit 0
}

Enter-Msvc
$preset = "windows-$($Config.ToLower())"
Invoke-Step 'Configure' { cmake --preset $preset @extraArgs }
Invoke-Step "Build ($Config)" { cmake --build --preset $preset }
if (-not $SkipTests)
{
    Invoke-Step 'Unit tests' { ctest --preset $preset }
}

$binDir = Join-Path $PSScriptRoot "build\$preset\bin"
Write-Host ""
Write-Host "Done. Binaries are in build\$preset\bin." -ForegroundColor Green
if ($Run)
{
    # Run from the exe's folder so it finds the assets and config copied next to it
    Push-Location $binDir
    try
    {
        & (Join-Path $binDir ($(if ($Sandbox) { 'sandbox.exe' } else { 'app.exe' })))
    }
    finally
    {
        Pop-Location
    }
}
