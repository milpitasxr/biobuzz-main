$ErrorActionPreference = 'Stop'
$adbPath = Join-Path $env:LOCALAPPDATA 'Android\Sdk\platform-tools\adb.exe'
if (!(Test-Path -LiteralPath $adbPath)) { throw 'Install Android SDK platform-tools first.' }
$devices = @(& $adbPath devices | Select-String '^\S+\s+device$' | ForEach-Object { ($_.Line -split '\s+')[0] })
if ($devices.Count -ne 1) { throw 'Connect exactly one authorized Control Hub over USB.' }
& $adbPath -s $devices[0] forward tcp:8080 tcp:8080
& $adbPath -s $devices[0] forward tcp:8000 tcp:8000
& $adbPath -s $devices[0] shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper
Start-Process 'http://localhost:8080/dash'
Write-Host 'Choose BIOBUZZ Vision Test and INIT. Camera and telemetry work during INIT; START is optional.'
