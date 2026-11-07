$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$jdk = Join-Path $env:USERPROFILE '.jdks\jbr-21.0.11'
if (Test-Path -LiteralPath $jdk) { $env:JAVA_HOME = $jdk }
& .\gradlew.bat :TeamCode:assembleDebug
if ($LASTEXITCODE -ne 0) { throw 'Build failed; nothing installed.' }
$adbPath = Join-Path $env:LOCALAPPDATA 'Android\Sdk\platform-tools\adb.exe'
$devices = @(& $adbPath devices | Select-String '^\S+\s+device$' | ForEach-Object { ($_.Line -split '\s+')[0] })
if ($devices.Count -ne 1) { throw 'Connect exactly one authorized Control Hub over USB.' }
& $adbPath -s $devices[0] install -r 'TeamCode\build\outputs\apk\debug\TeamCode-debug.apk'
if ($LASTEXITCODE -ne 0) { throw 'Installation failed. Existing app was not uninstalled.' }
& .\open-vision-dashboard.ps1
