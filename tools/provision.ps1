<#
.SYNOPSIS
  Write Wi-Fi/MQTT credentials to the device's "creds" partition over USB.

.DESCRIPTION
  Credentials are not compiled into the firmware. This script turns
  config/creds.csv (git-ignored; see config/creds.csv.template) into an NVS
  image with ESP-IDF's nvs_partition_gen, flashes it to the creds partition
  offset from partitions.csv, and deletes the temporary image. OTA updates
  never touch this partition, so run it again only when credentials change.

  The device resets afterwards and connects with the new credentials.

.EXAMPLE
  ./tools/provision.ps1 -Port COM3
#>
[CmdletBinding()]
param(
    [string]$Port = "COM3",
    [string]$CredsFile
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot
if (-not $CredsFile) { $CredsFile = Join-Path $Root "config\creds.csv" }
if (-not (Test-Path $CredsFile)) {
    throw "No $CredsFile - copy config/creds.csv.template and fill it in."
}

# Partition offset/size come from partitions.csv so they cannot drift.
$row = Get-Content (Join-Path $Root "partitions.csv") |
    Where-Object { $_ -match '^\s*creds\s*,' } | Select-Object -First 1
if (-not $row) { throw "No 'creds' partition in partitions.csv" }
$fields = $row -split ',' | ForEach-Object { $_.Trim() }
$offset = $fields[3]
$size = $fields[4]

# Validate before generating anything.
$entries = Get-Content $CredsFile | Where-Object { $_ -notmatch '^\s*#' } | ConvertFrom-Csv
$ns = $entries | Where-Object { $_.type -eq 'namespace' }
if (@($ns).Count -ne 1 -or $ns.key -ne 'creds') { throw "creds.csv must declare exactly one namespace named 'creds'" }
$values = @{}
foreach ($e in $entries | Where-Object { $_.type -eq 'data' }) {
    if ($e.encoding -ne 'string') { throw "Key '$($e.key)' must use encoding 'string'" }
    $values[$e.key] = $e.value
}
$limits = @{ wifi_ssid = 32; wifi_pass = 64; mqtt_user = 64; mqtt_pass = 128 }
foreach ($k in $values.Keys) {
    if (-not $limits.ContainsKey($k)) { throw "Unknown key '$k' (expected: $($limits.Keys -join ', '))" }
    if ([Text.Encoding]::UTF8.GetByteCount($values[$k]) -gt $limits[$k]) { throw "'$k' is longer than $($limits[$k]) bytes" }
}
foreach ($k in 'wifi_ssid', 'wifi_pass') {
    if (-not $values[$k]) { throw "'$k' is required" }
}
if ($values['wifi_pass'].Length -lt 8) { throw "'wifi_pass' must be at least 8 characters (WPA2)" }
if ($values['mqtt_user'] -and -not $values['mqtt_pass']) { throw "'mqtt_user' is set but 'mqtt_pass' is empty" }

$python = if ($env:IDF_PYTHON_ENV_PATH) { Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\python.exe" } else { "python" }

$tmp = Join-Path ([IO.Path]::GetTempPath()) ("ld2420-creds-" + [guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $tmp | Out-Null
try {
    $image = Join-Path $tmp "creds.bin"
    & $python -m esp_idf_nvs_partition_gen generate $CredsFile $image $size | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "nvs_partition_gen failed" }

    Write-Host "Writing credentials ($size bytes) to $offset on $Port"
    & $python -m esptool --chip esp32c3 -p $Port write_flash $offset $image
    if ($LASTEXITCODE -ne 0) { throw "esptool write_flash failed" }
    Write-Host "Done. The device restarts and connects with the provisioned credentials."
}
finally {
    Remove-Item -Recurse -Force $tmp -ErrorAction SilentlyContinue
}
