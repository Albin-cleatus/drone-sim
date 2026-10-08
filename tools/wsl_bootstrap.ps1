<#
    wsl_bootstrap.ps1 - one-time, ELEVATED setup for WSL2 on Windows.

    Keep this file pure ASCII. PowerShell 5.1 parses a BOM-less script using the
    ANSI codepage, so a UTF-8 em-dash decodes to 0x94, which it then reads as a
    smart-quote string delimiter and dies on with a bogus "missing terminator".

    Enables the two Windows optional features WSL2 requires and installs the WSL
    launcher/kernel. It deliberately does NOT install a distribution: registering
    a distro does not need Administrator, and keeping it separate means this
    script needs at most one reboot.

    From an Administrator PowerShell:

        powershell -ExecutionPolicy Bypass -File tools\wsl_bootstrap.ps1

    Then reboot if it says so, then register Ubuntu (no admin needed):

        wsl --install -d Ubuntu-24.04 --no-launch

    --no-launch matters: it registers the distro without running its first-run
    wizard, so you never have to type a UNIX username/password. Use
    tools/provision_ardupilot.sh to create the build user instead.
#>

[CmdletBinding()]
param(
    # Skip the `wsl --install` step and only toggle the Windows features.
    [switch]$FeaturesOnly
)

$ErrorActionPreference = 'Stop'

# wsl.exe emits UTF-16LE when its output is redirected, which reads back as
# spaced-out garbage through a pipe. This makes it speak UTF-8 instead.
$env:WSL_UTF8 = '1'

$LogPath    = Join-Path $PSScriptRoot 'wsl_bootstrap.log'
$Transcript = New-Object System.Collections.Generic.List[string]

function Say {
    param([string]$Message)
    $line = "[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $Message
    Write-Host $line
    $Transcript.Add($line)
    # Write as we go: if the script dies mid-way the log still explains how far
    # it got, which is the whole point of keeping one.
    try { $Transcript | Set-Content -Path $LogPath -Encoding utf8 } catch { }
}

function Get-FeatureState {
    param([string]$Name)
    (Get-WindowsOptionalFeature -Online -FeatureName $Name -ErrorAction Stop).State
}

# Every step below needs Administrator, and failing halfway would leave the
# features inconsistent, so refuse up front rather than partway through.
$identity  = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Say "ERROR: not elevated. Re-run from an Administrator PowerShell:"
    Say "       powershell -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit 1
}

Say "elevated: yes"
Say ("Windows: {0} ({1})" -f (Get-CimInstance Win32_OperatingSystem).Caption,
                              (Get-CimInstance Win32_OperatingSystem).Version)

$cpu = Get-CimInstance Win32_Processor | Select-Object -First 1
Say ("virtualisation firmware enabled: {0}; hypervisor present: {1}" -f
     $cpu.VirtualizationFirmwareEnabled, (Get-CimInstance Win32_ComputerSystem).HypervisorPresent)

$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Say "free space on C: $freeGb GB"
if ($freeGb -lt 20) {
    Say "WARNING: under 20 GB free. ArduPilot's build needs roughly 5-8 GB,"
    Say "         and the Ubuntu distro's virtual disk grows to a few more."
}

$needsRestart = $false

foreach ($feature in @('Microsoft-Windows-Subsystem-Linux', 'VirtualMachinePlatform')) {
    $state = Get-FeatureState $feature
    Say "$feature is $state"

    if ($state -eq 'Enabled') {
        Say "  already enabled, nothing to do"
        continue
    }

    Say "  enabling (this is the part that needs Administrator)..."
    $result = Enable-WindowsOptionalFeature -Online -FeatureName $feature -All -NoRestart
    # RestartNeeded is a RestartType enum on some DISM builds and a plain bool on
    # others. Comparing the enum against the *string* 'No' silently never fires:
    # PowerShell coerces a non-empty string to $true, so the test becomes
    # $true -ne $true. Stringify it first and compare against the known-not-set
    # spellings instead.
    $restartNeeded = "$($result.RestartNeeded)"
    Say "  -> now $(Get-FeatureState $feature); RestartNeeded=$restartNeeded"
    if ($restartNeeded -notin @('No', 'False', '0')) { $needsRestart = $true }
}

if ($FeaturesOnly) {
    Say "-FeaturesOnly given; stopping before the WSL install."
} else {
    Say "installing the WSL launcher and kernel (no distribution)..."
    # --no-distribution keeps this to the platform pieces. Exit codes here are
    # unreliable pre-reboot, so the version probe below is the real check.
    & wsl.exe --install --no-distribution 2>&1 | ForEach-Object { Say "  $_" }
    Say "  wsl --install exit code: $LASTEXITCODE"

    if ($needsRestart) {
        Say "  a restart is pending, so the launcher may not be usable yet"
    } else {
        Say "checking the WSL version..."
        & wsl.exe --version 2>&1 | ForEach-Object { Say "  $_" }
        & wsl.exe --set-default-version 2 2>&1 | ForEach-Object { Say "  $_" }
    }
}

Say ""
if ($needsRestart) {
    Say "RESULT: Windows features changed - RESTART REQUIRED."
    Say "After rebooting, register Ubuntu (Admin not needed):"
    Say "    wsl --install -d Ubuntu-24.04 --no-launch"
} else {
    Say "RESULT: no restart needed."
    Say "Register Ubuntu (Admin not needed):"
    Say "    wsl --install -d Ubuntu-24.04 --no-launch"
}
Say "Log written to $LogPath"

exit 0
