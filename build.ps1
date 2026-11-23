# Builds CppTemplate on Windows after install.bat has run once. The README's Command line section lists the
# options.
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Config = 'Debug',
    [switch]$Web,
    [switch]$Run,
    [switch]$Sandbox,
    [switch]$Physics2D,
    [switch]$SkipTests,
    [switch]$GpuCompute
)

$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

# Picks up tools installed after this window opened, so nothing needs restarting after install.bat
$env:Path = @([Environment]::GetEnvironmentVariable('Path', 'Machine'), [Environment]::GetEnvironmentVariable('Path', 'User'), $env:Path) -join ';'

# Repeated runs in one window would otherwise keep growing PATH until cmd.exe, and with it emsdk, can't read it
$seenPaths = @{}
$uniquePaths = foreach ($entry in $env:Path -split ';')
{
    $key = $entry.TrimEnd('\')
    if ($key -and -not $seenPaths.ContainsKey($key))
    {
        $seenPaths[$key] = $true
        $entry
    }
}
$env:Path = $uniquePaths -join ';'

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

# The MSVC cl.exe a build folder was configured with, if it still exists
function Get-CachedMsvc([string]$BuildDir)
{
    $cache = Join-Path $BuildDir 'CMakeCache.txt'
    if (-not (Test-Path $cache))
    {
        return $null
    }
    $line = Select-String -Path $cache -Pattern '^CMAKE_CXX_COMPILER:FILEPATH=(.+)$' | Select-Object -First 1
    if (-not $line)
    {
        return $null
    }
    $compiler = [IO.Path]::GetFullPath($line.Matches[0].Groups[1].Value.Trim())
    if ($compiler -match '\\VC\\Tools\\MSVC\\' -and (Test-Path $compiler))
    {
        return $compiler
    }
    return $null
}

# The MSVC used by the build folders CLion has enabled, so both share one compiler
function Get-ClionMsvc
{
    $workspace = Join-Path $PSScriptRoot '.idea\workspace.xml'
    if (-not (Test-Path $workspace))
    {
        return $null
    }
    $profiles = Select-String -Path $workspace -Pattern '<configuration [^>]*ENABLED="true"[^>]*GENERATION_DIR="([^"]+)"'
    foreach ($clionProfile in $profiles)
    {
        $dir = $clionProfile.Matches[0].Groups[1].Value.Replace('$PROJECT_DIR$', $PSScriptRoot)
        $compiler = Get-CachedMsvc $dir
        if ($compiler)
        {
            return $compiler
        }
    }
    return $null
}

# Loads the compiler environment, preferring the MSVC that CLion or this preset's build folder already uses, then the
# newest Visual Studio with the C++ tools. Returns the cl.exe that ends up on PATH.
function Enter-Msvc([string]$BuildDir)
{
    $wanted = Get-ClionMsvc
    if (-not $wanted)
    {
        $wanted = Get-CachedMsvc $BuildDir
    }
    $current = Get-Command cl.exe -ErrorAction SilentlyContinue
    if ($current -and (-not $wanted -or $current.Source -eq $wanted))
    {
        return $current.Source
    }

    $vcvarsArgs = ''
    if ($wanted)
    {
        $vsPath = $wanted.Substring(0, $wanted.IndexOf('\VC\Tools\MSVC\'))
        $toolsVersion = ($wanted.Substring($vsPath.Length + '\VC\Tools\MSVC\'.Length) -split '\\')[0]
        $vcvarsArgs = "-vcvars_ver=$toolsVersion"
        Write-Host "Using MSVC $toolsVersion from $vsPath, which the existing build already uses"
    }
    else
    {
        $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
        if (-not (Test-Path $vswhere))
        {
            throw 'Visual Studio was not found. Run install.bat first.'
        }
        $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1
    }
    $vcvars = if ($vsPath) { Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat' } else { $null }
    if (-not $vcvars -or -not (Test-Path $vcvars))
    {
        throw 'The Visual Studio C++ tools were not found. Run install.bat first.'
    }
    foreach ($line in (& cmd.exe /c "`"$vcvars`" $vcvarsArgs >nul && set"))
    {
        $name, $value = $line -split '=', 2
        if ($value)
        {
            Set-Item -Path "env:$name" -Value $value
        }
    }
    return (Get-Command cl.exe).Source
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
        $page = if ($Sandbox) { 'sandbox.html' } elseif ($Physics2D) { 'physics2d.html' } else { '' }
        Start-Process "http://localhost:8080/$page"
        & python -m http.server 8080 --directory build\web\bin
    }
    exit 0
}

$preset = "windows-$($Config.ToLower())"
$buildDir = Join-Path $PSScriptRoot "build\$preset"
$compiler = Enter-Msvc $buildDir

# A build folder set up with another MSVC would mix two compilers, so it starts over with this one
$cachedCompiler = Get-CachedMsvc $buildDir
if ($cachedCompiler -and $cachedCompiler -ne $compiler)
{
    Write-Host "build\$preset was set up with $cachedCompiler, so it is reconfigured with $compiler"
    Remove-Item (Join-Path $buildDir 'CMakeCache.txt'), (Join-Path $buildDir 'CMakeFiles') -Recurse -Force -ErrorAction SilentlyContinue
}
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
        $exe = if ($Sandbox) { 'sandbox.exe' } elseif ($Physics2D) { 'physics2d.exe' } else { 'app.exe' }
        & (Join-Path $binDir $exe)
    }
    finally
    {
        Pop-Location
    }
}
