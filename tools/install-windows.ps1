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
    & winget install --id $Id --exact --silent --disable-interactivity --accept-package-agreements --accept-source-agreements @Extra
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

# The newest Visual Studio of any edition, whether or not it has the C++ tools
function Find-AnyVisualStudio
{
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere))
    {
        return $null
    }
    $json = (& $vswhere -latest -products * -format json) -join "`n"
    return (ConvertFrom-Json $json) | Select-Object -First 1
}

# Adds the C++ workload to an existing install through the Visual Studio Installer, which asks for admin rights
function Add-CppWorkload($Instance)
{
    $setup = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\setup.exe'
    $workload = if ($Instance.productId -eq 'Microsoft.VisualStudio.Product.BuildTools') { 'Microsoft.VisualStudio.Workload.VCTools' } else { 'Microsoft.VisualStudio.Workload.NativeDesktop' }
    Write-Host "  Adding the C++ workload to $($Instance.displayName) (close Visual Studio first if it's open)..." -ForegroundColor Cyan
    $arguments = @('modify', '--installPath', "`"$($Instance.installationPath)`"", '--add', $workload, '--includeRecommended', '--quiet', '--norestart')
    $process = Start-Process $setup -ArgumentList $arguments -Verb RunAs -Wait -PassThru
    # 3010 means it worked but Windows wants a restart
    if ($process.ExitCode -ne 0 -and $process.ExitCode -ne 3010)
    {
        Write-Host "  The Visual Studio Installer failed (exit $($process.ExitCode)). Add the C++ workload from the Visual Studio Installer instead." -ForegroundColor Red
    }
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

function Get-EmscriptenVersion([string]$Dir)
{
    $versionFile = if ($Dir) { Join-Path $Dir 'upstream\emscripten\emscripten-version.txt' } else { '' }
    if ($versionFile -and (Test-Path $versionFile))
    {
        return (Get-Content $versionFile -Raw).Trim().Trim('"')
    }
    return ''
}

# Fetches emsdk itself, with git when it's already here, otherwise as a zip so it needn't wait for Git to install
function Get-EmsdkSource([string]$Dir)
{
    Write-Host "  Downloading emsdk into $Dir..." -ForegroundColor Cyan
    if (Get-Command git -ErrorAction SilentlyContinue)
    {
        & git clone --depth 1 --quiet https://github.com/emscripten-core/emsdk.git $Dir
        if ($LASTEXITCODE -ne 0)
        {
            throw "git clone emsdk failed (exit $LASTEXITCODE)"
        }
        return
    }
    $zip = Join-Path $env:TEMP 'emsdk-main.zip'
    $unpacked = Join-Path $env:TEMP 'emsdk-unpacked'
    $ProgressPreference = 'SilentlyContinue' # The progress bar slows Invoke-WebRequest down many times over
    Invoke-WebRequest 'https://github.com/emscripten-core/emsdk/archive/refs/heads/main.zip' -OutFile $zip -UseBasicParsing
    Remove-Item $unpacked -Recurse -Force -ErrorAction SilentlyContinue
    Expand-Archive $zip $unpacked
    New-Item -ItemType Directory -Force (Split-Path $Dir) | Out-Null
    Move-Item (Join-Path $unpacked 'emsdk-main') $Dir
    Remove-Item $zip, $unpacked -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host ""
Write-Host "CppTemplate dependencies" -ForegroundColor Cyan

# Emscripten is the slowest download, so it installs in the background while winget works through the rest
$emsdkJob = $null
$emsdkLog = Join-Path $env:TEMP 'cpptemplate-emsdk-install.log'
if (-not $NoWeb)
{
    $emsdk = Find-Emsdk
    if (-not $emsdk -and -not $CheckOnly)
    {
        $emsdk = if ($EmsdkDir) { $EmsdkDir } else { Join-Path $HOME 'tools\emsdk' }
        Get-EmsdkSource $emsdk
    }
    if ($emsdk -and (Get-EmscriptenVersion $emsdk) -ne $EmsdkVersion -and -not $CheckOnly)
    {
        Write-Host "  Installing Emscripten $EmsdkVersion in the background (log: $emsdkLog)..." -ForegroundColor Cyan
        $emsdkBat = Join-Path $emsdk 'emsdk.bat'
        $emsdkJob = Start-Process cmd.exe -ArgumentList '/c', "`"`"$emsdkBat`" install $EmsdkVersion && `"$emsdkBat`" activate $EmsdkVersion`"" `
            -RedirectStandardOutput $emsdkLog -RedirectStandardError "$emsdkLog.err" -NoNewWindow -PassThru
        $null = $emsdkJob.Handle # Keeps the handle open so the exit code can be read afterwards
    }
}

# Windows Installer runs one install at a time, so winget installs stay in sequence
Ensure-Tool 'Git' 'git' 'Git.Git'
Ensure-Tool 'CMake' 'cmake' 'Kitware.CMake'
Ensure-Tool 'Ninja' 'ninja' 'Ninja-build.Ninja'
Ensure-Tool 'Python 3' 'python' 'Python.Python.3.12'

$vsPath = Find-VisualStudio
$vsWithoutCpp = if ($vsPath) { $null } else { Find-AnyVisualStudio }
if ($vsWithoutCpp -and -not $CheckOnly)
{
    Add-CppWorkload $vsWithoutCpp
    $vsPath = Find-VisualStudio
}
elseif (-not $vsPath -and -not $CheckOnly)
{
    Install-WithWinget 'Microsoft.VisualStudio.2022.BuildTools' @('--override', '--quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended')
    $vsPath = Find-VisualStudio
}
$vsMissing = if ($vsWithoutCpp) { "C++ workload for $($vsWithoutCpp.installationPath)" } else { 'Visual Studio 2022 Build Tools, C++ workload' }
Write-Status 'MSVC (Visual Studio)' ([bool]$vsPath) ($(if ($vsPath) { $vsPath } else { $vsMissing }))

if (-not $NoWeb)
{
    if ($emsdkJob)
    {
        if (-not $emsdkJob.HasExited)
        {
            Write-Host "  Waiting for the Emscripten install to finish..." -ForegroundColor Cyan
        }
        $emsdkJob.WaitForExit()
        if ($emsdkJob.ExitCode -ne 0)
        {
            Get-Content $emsdkLog, "$emsdkLog.err" -Tail 20 -ErrorAction SilentlyContinue | Write-Host
            Write-Host "  The Emscripten install failed (exit $($emsdkJob.ExitCode)). Full log: $emsdkLog" -ForegroundColor Red
        }
    }
    $installed = Get-EmscriptenVersion $emsdk

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
