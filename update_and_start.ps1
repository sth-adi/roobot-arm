# Stops any running copy of this app, pulls the latest code from GitHub (branch rarm-core) and starts it again.
# Only Python processes running this app's scripts are stopped (launcher, hub, server, controller client, emulator, proxy), not every python.exe.
Set-Location -Path $PSScriptRoot
Write-Host "Stopping old copies of the arm hub..." -ForegroundColor Cyan
$pattern = 'launch\.py|dashboard[\\/]hub\.py|[\\/ ]server\.py|controller_client|controller_gui|emu[\\/]netproxy|emu[\\/]emulator'
Get-CimInstance Win32_Process | Where-Object {
    $_.ProcessId -ne $PID -and $_.Name -match '^(python|pythonw|py)(\d|\.exe)*' -and $_.CommandLine -match $pattern
} | ForEach-Object {
    Write-Host ("  stopping {0} (pid {1})" -f $_.Name, $_.ProcessId)
    Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
}
Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.Name -like 'esp32_*' } | Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 1

Write-Host "Pulling the latest code..." -ForegroundColor Cyan
git fetch origin
git checkout rarm-core
git pull --ff-only origin rarm-core
if ($LASTEXITCODE -ne 0) {
    Write-Host "git pull did not finish (local changes in the way?). Fix that, then run this again." -ForegroundColor Red
    exit 1
}
git log --oneline -1

$py = Join-Path $env:USERPROFILE "anaconda3\python.exe"
if (-not (Test-Path $py)) { $py = "python" }
Write-Host "Starting..." -ForegroundColor Cyan
& $py launch.py
