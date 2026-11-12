# Installs whatever CppTemplate needs on Windows 10/11 with winget and emsdk. Safe to re-run; -CheckOnly only
# reports, and -NoWeb skips emsdk.
param(
    [switch]$CheckOnly,
    [switch]$NoWeb,
    [string]$EmsdkVersion = '5.0.4',
    [string]$EmsdkDir = ''
)

$ErrorActionPreference = 'Stop'
$script:missing = @()

function Write-Status([string]$Name, [bool]$Found, [string]$Detail)
{
    if ($Found)
    {
        Write-Host ("  [ok]      {0,-22} {1}" -f $Name, $Detail) -ForegroundColor Green
    }
    else
    {
        Write-Host ("  [missing] {0,-22} {1}" -f $Name, $Detail) -ForegroundColor Yellow
        $script:missing += $Name
    }
}

# winget installs land on the machine PATH, which this session hasn't seen yet
function Update-SessionPath
{
    $machine = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    $user = [Environment]::GetEnvironmentVariable('Path', 'User')
    $env:Path = "$machine;$user"
}

function Install-WithWinget([string]$Id, [string[]]$Extra = @())
{
    if (-not (Get-Command winget -ErrorAction SilentlyContinue))
    {
        throw "winget is not available. Install 'App Installer' from the Microsoft Store, then re-run."
    }
    Write-Host "  Installing $Id with winget..." -ForegroundColor Cyan
    & winget install --id $Id --exact --silent --accept-package-agreements --accept-source-agreements @Extra
    # 0 is success; -1978335189 means it was already installed
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne -1978335189)
    {
        throw "winget install $Id failed (exit $LASTEXITCODE)"
    }
    Update-SessionPath
}

function Get-ToolVersion([string]$Command, [string]$VersionArg = '--version')
{
    $tool = Get-Command $Command -ErrorAction SilentlyContinue
    if (-not $tool)
    {
        return $null
    }
    return ((& $tool.Source $VersionArg 2>$null) | Select-Object -First 1)
}

function Ensure-Tool([string]$Name, [string]$Command, [string]$WingetId)
{
    $version = Get-ToolVersion $Command
    if (-not $version -and -not $CheckOnly)
    {
        Install-WithWinget $WingetId
        $version = Get-ToolVersion $Command
    }
    Write-Status $Name ([bool]$version) ($(if ($version) { $version } else { "winget id $WingetId" }))
}

function Find-VisualStudio
{
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere))
    {
        return $null
    }
    return (& $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1)
}

function Find-Emsdk
{
    $candidates = @($EmsdkDir, $env:EMSDK, [Environment]::GetEnvironmentVariable('EMSDK', 'User'), 'C:\emsdk', (Join-Path $HOME 'tools\emsdk'))
    foreach ($dir in $candidates)
    {
        if ($dir -and (Test-Path (Join-Path $dir 'emsdk.ps1')))
        {
            return $dir
        }
    }
    return $null
}

Write-Host ""
Write-Host "CppTemplate dependencies" -ForegroundColor Cyan

Ensure-Tool 'Git' 'git' 'Git.Git'
Ensure-Tool 'CMake' 'cmake' 'Kitware.CMake'
Ensure-Tool 'Ninja' 'ninja' 'Ninja-build.Ninja'
Ensure-Tool 'Python 3' 'python' 'Python.Python.3.12'

$vsPath = Find-VisualStudio
if (-not $vsPath -and -not $CheckOnly)
{
    Install-WithWinget 'Microsoft.VisualStudio.2022.BuildTools' @('--override', '--quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended')
    $vsPath = Find-VisualStudio
}
Write-Status 'MSVC (Visual Studio)' ([bool]$vsPath) ($(if ($vsPath) { $vsPath } else { 'Visual Studio 2022 Build Tools, C++ workload' }))

if (-not $NoWeb)
{
    $emsdk = Find-Emsdk
    if (-not $emsdk -and -not $CheckOnly)
    {
        $emsdk = if ($EmsdkDir) { $EmsdkDir } else { Join-Path $HOME 'tools\emsdk' }
        Write-Host "  Cloning emsdk into $emsdk..." -ForegroundColor Cyan
        & git clone --depth 1 https://github.com/emscripten-core/emsdk.git $emsdk
        if ($LASTEXITCODE -ne 0)
        {
            throw "git clone emsdk failed (exit $LASTEXITCODE)"
        }
    }

    $versionFile = if ($emsdk) { Join-Path $emsdk 'upstream\emscripten\emscripten-version.txt' } else { '' }
    $installed = if ($versionFile -and (Test-Path $versionFile)) { (Get-Content $versionFile -Raw).Trim().Trim('"') } else { '' }
    if ($emsdk -and $installed -ne $EmsdkVersion -and -not $CheckOnly)
    {
        Write-Host "  Installing Emscripten $EmsdkVersion (a few minutes)..." -ForegroundColor Cyan
        & (Join-Path $emsdk 'emsdk.ps1') install $EmsdkVersion
        & (Join-Path $emsdk 'emsdk.ps1') activate $EmsdkVersion
        $installed = (Get-Content $versionFile -Raw).Trim().Trim('"')
    }

    if ($emsdk -and -not $CheckOnly -and [Environment]::GetEnvironmentVariable('EMSDK', 'User') -ne $emsdk)
    {
        # The web preset's toolchain file reads this saved setting, so open IDEs see it without a restart
        [Environment]::SetEnvironmentVariable('EMSDK', $emsdk, 'User')
        $env:EMSDK = $emsdk
    }

    $detail = if ($emsdk) { "$emsdk (Emscripten $installed, want $EmsdkVersion)" } else { "emsdk $EmsdkVersion" }
    Write-Status 'Emscripten (emsdk)' ([bool]($emsdk -and $installed -eq $EmsdkVersion)) $detail
    $emsdkUserVar = [Environment]::GetEnvironmentVariable('EMSDK', 'User')
    Write-Status 'EMSDK variable' ([bool]$emsdkUserVar) ($(if ($emsdkUserVar) { $emsdkUserVar } else { 'set for your user so the web preset can find emsdk' }))
}

Write-Host ""
if ($script:missing.Count -gt 0)
{
    if ($CheckOnly)
    {
        Write-Host "Missing: $($script:missing -join ', '). Run without -CheckOnly to install." -ForegroundColor Yellow
    }
    else
    {
        Write-Host "Still missing: $($script:missing -join ', '). See the messages above." -ForegroundColor Red
        exit 1
    }
}
else
{
    Write-Host "Everything is installed. Next: run.bat (or open the folder in CLion and pick a preset)." -ForegroundColor Green
}
