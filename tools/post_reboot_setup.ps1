<#
    post_reboot_setup.ps1 - finish the WSL2 + ArduPilot setup after the reboot.

    Registered as an HKCU RunOnce entry, so it runs once at logon. Nothing here
    needs Administrator: enabling the Windows features was the only privileged
    step, and that is already done.

    In order:
      1. wait for the network - the distro download needs it, and RunOnce can
         fire before the network is up
      2. register the Ubuntu-24.04 distro, if it is not registered already
      3. run tools/provision_ardupilot.sh inside it as root - the long part,
         the 10-30 minute ArduPilot Copter build

    Everything is appended to logs/post_reboot_setup.log, and the last banner in
    that file says whether it worked. Both steps are idempotent, so re-running
    this script by hand is safe - which matters, because RunOnce values are
    deleted before the command runs, so a failure does not retry itself.

    Keep this file pure ASCII. PowerShell 5.1 parses a BOM-less script using the
    ANSI codepage, where a UTF-8 em-dash decodes to 0x94 and is then read as a
    smart-quote string delimiter, producing a bogus "missing terminator" error.
#>

[CmdletBinding()]
param(
    [string]$Root            = 'C:\Users\Albin\drone-sim',
    [string]$Distro          = 'Ubuntu-24.04',
    [string]$BuildUser       = 'albin',
    [int]$NetworkWaitSeconds = 600,
    [string]$RunOnceName     = 'ArduPilotWSLSetup'
)

$ErrorActionPreference = 'Continue'
$env:WSL_UTF8 = '1'     # otherwise wsl.exe emits UTF-16LE through a pipe

$LogDir    = Join-Path $Root 'logs'
$Log       = Join-Path $LogDir 'post_reboot_setup.log'
$ResultTxt = Join-Path $LogDir 'post_reboot_setup.result.txt'
$Provision = Join-Path $Root 'tools\provision_ardupilot.sh'

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

# A StreamWriter held open for the run instead of Add-Content per line: the
# ArduPilot build emits tens of thousands of lines, and reopening the file for
# each one would add minutes. AutoFlush keeps it readable while it runs.
$script:LogWriter = New-Object System.IO.StreamWriter(
    $Log, $true, (New-Object System.Text.UTF8Encoding($false)))
$script:LogWriter.AutoFlush = $true

$script:Failed = $false
$step = 'startup'

function Write-Log {
    param([string]$Message, [string]$Level = 'INFO')
    $line = '[{0}] {1,-5} {2}' -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Level, "$Message"
    Write-Host $line
    try { $script:LogWriter.WriteLine($line) } catch { }
}

function Write-Banner {
    param([string]$Text)
    Write-Log ('=' * 66)
    Write-Log $Text
    Write-Log ('=' * 66)
}

function ConvertTo-WslPath {
    param([string]$WindowsPath)
    $p = $WindowsPath -replace '\\', '/'
    if ($p -match '^([A-Za-z]):(.*)$') { return '/mnt/' + $Matches[1].ToLower() + $Matches[2] }
    return $p
}

function Wait-Network {
    param([int]$TimeoutSeconds)
    Write-Log "waiting for network access (up to ${TimeoutSeconds}s)"
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        $client = $null
        try {
            $client = New-Object System.Net.Sockets.TcpClient
            $iar = $client.BeginConnect('github.com', 443, $null, $null)
            if ($iar.AsyncWaitHandle.WaitOne(5000) -and $client.Connected) {
                Write-Log 'network is up'
                return $true
            }
        } catch {
        } finally {
            if ($client) { $client.Close() }
        }
        Start-Sleep -Seconds 5
    }
    Write-Log "network never came up within ${TimeoutSeconds}s" 'ERROR'
    return $false
}

function Invoke-Wsl {
    param([string[]]$WslArgs, [string]$What)
    Write-Log ("running: wsl.exe " + ($WslArgs -join ' '))
    & wsl.exe @WslArgs 2>&1 | ForEach-Object { Write-Log "  $_" }
    $code = $LASTEXITCODE
    if ($code -ne 0) { Write-Log "$What exited with code $code" 'WARN' }
    return $code
}

function Get-Distros {
    $raw = & wsl.exe --list --quiet 2>&1
    # Strip NULs defensively: some WSL builds still pad the output.
    return @($raw | ForEach-Object { ("$_" -replace "`0", '').Trim() } | Where-Object { $_ -ne '' })
}

# --------------------------------------------------------------------------- #

Write-Banner 'ArduPilot SITL post-reboot setup starting'
Write-Log "root          : $Root"
Write-Log "distro        : $Distro"
Write-Log ("uptime        : {0:N1} minutes" -f `
    ((Get-Date) - (Get-CimInstance Win32_OperatingSystem).LastBootUpTime).TotalMinutes)

# RunOnce normally deletes the value before running the command, but clear ours
# explicitly so that a partial failure cannot turn into a run-at-every-logon loop.
try {
    Remove-ItemProperty -Path 'HKCU:\Software\Microsoft\Windows\CurrentVersion\RunOnce' `
        -Name $RunOnceName -ErrorAction SilentlyContinue
    Write-Log 'cleared the RunOnce entry'
} catch {
    Write-Log "could not clear the RunOnce entry: $($_.Exception.Message)" 'WARN'
}

try {
    $step = 'network'
    if (-not (Wait-Network -TimeoutSeconds $NetworkWaitSeconds)) {
        throw 'no network access'
    }

    $step = 'wsl availability'
    Write-Log 'checking wsl --status'
    & wsl.exe --status 2>&1 | ForEach-Object { Write-Log "  $_" }
    & wsl.exe --version 2>&1 | ForEach-Object { Write-Log "  $_" }

    $step = 'distro registration'
    $distros = Get-Distros
    Write-Log ("registered distros: [" + ($distros -join ', ') + "]")

    if ($distros -notcontains $Distro) {
        Write-Log "$Distro is not registered; installing it (this downloads the image)"
        Invoke-Wsl -WslArgs @('--install', '-d', $Distro, '--no-launch') -What 'wsl --install' | Out-Null

        # Registration can lag the command returning, so poll rather than assume.
        for ($i = 1; $i -le 24; $i++) {
            Start-Sleep -Seconds 5
            if ((Get-Distros) -contains $Distro) { break }
            Write-Log "  not registered yet (attempt $i/24)"
        }
    } else {
        Write-Log "$Distro is already registered"
    }

    if ((Get-Distros) -notcontains $Distro) {
        throw "$Distro still not registered after waiting - a reboot may still be pending"
    }
    Write-Log "$Distro is registered"

    $step = 'distro smoke test'
    Invoke-Wsl -WslArgs @('-d', $Distro, '-u', 'root', '--', '/bin/sh', '-c',
        'uname -a; head -2 /etc/os-release; nproc; free -m | head -2') -What 'smoke test' | Out-Null

    $step = 'ArduPilot build'
    $wslProvision = ConvertTo-WslPath $Provision
    Write-Log "provisioning via $wslProvision"
    Write-Log 'this is the long part: expect 10-30 minutes of compiler output'

    $code = Invoke-Wsl -WslArgs @('-d', $Distro, '-u', 'root', '--', 'bash', $wslProvision) `
                       -What 'provision_ardupilot.sh'
    if ($code -ne 0) { throw "provision_ardupilot.sh exited with code $code" }

    $step = 'verification'
    $binary = "/home/$BuildUser/ardupilot/build/sitl/bin/arducopter"
    $code = Invoke-Wsl -WslArgs @('-d', $Distro, '-u', $BuildUser, '--', '/bin/sh', '-c',
        "test -x '$binary' && ls -la '$binary' && echo BINARY_OK") -What 'verify SITL binary'
    if ($code -ne 0) { throw "SITL binary missing or not executable at $binary" }

} catch {
    $script:Failed = $true
    Write-Log "FAILED during step '$step': $($_.Exception.Message)" 'ERROR'
}

# --------------------------------------------------------------------------- #

$binaryUnix = "/home/$BuildUser/ardupilot/build/sitl/bin/arducopter"
$launchCmd  = "wsl -d $Distro -u $BuildUser -- bash -lc 'cd ~/ardupilot/ArduCopter && " +
              "exec sim_vehicle.py -v ArduCopter -f quad -L CMAC " +
              "--console --map --out=udp:127.0.0.1:14550'"

if ($script:Failed) {
    Write-Banner "RESULT: FAILED during step '$step'"
    Write-Log "see the log above; re-run by hand with:"
    Write-Log "  powershell -ExecutionPolicy Bypass -File '$PSCommandPath'"
    $summary = @(
        "FAILED during step: $step",
        "Log: $Log",
        "",
        "Re-run by hand (no admin needed):",
        "  powershell -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    )
} else {
    Write-Banner 'RESULT: SUCCESS - ArduPilot SITL is built'
    Write-Log "binary: $binaryUnix"
    Write-Log "start SITL with:"
    Write-Log "  $launchCmd"
    $summary = @(
        "SUCCESS - ArduPilot SITL is built",
        "Binary (inside WSL): $binaryUnix",
        "Log: $Log",
        "",
        "Start the simulator (Terminal 1):",
        "  $launchCmd",
        "",
        "Fly the mission (Terminal 2):",
        "  cd $Root",
        "  .\.venv\Scripts\Activate.ps1",
        "  python sitl_mission.py --connect udpin:0.0.0.0:14550"
    )
}

try {
    Set-Content -Path $ResultTxt -Value $summary -Encoding utf8
    Write-Log "result summary written to $ResultTxt"
    $script:LogWriter.Close()
} catch { }

exit $(if ($script:Failed) { 1 } else { 0 })
