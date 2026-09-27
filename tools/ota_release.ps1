<#
.SYNOPSIS
  Publish the current build as an OTA update for the LD2420 presence sensor.

.DESCRIPTION
  1. Checks build/presence_sensor_ld2420.bin is signed with the OTA key
     (tools/ota_signing_pubkey.pem) and carries the PROJECT_VER version.
  2. Copies it to Home Assistant under /config/www/ota/<device>/<random>/,
     replacing any previous release (served at <HaUrl>/local/...).
  3. Publishes a retained manifest {version,url,sha256,size,notes} to
     presence/<device>/cmd/ota/manifest over MQTT. HA then shows
     "Update available" on the device's Firmware entity; pressing Install
     makes the device download, verify (SHA-256 + signature) and reboot.

  -Clean removes the hosted file and clears the manifest (run it once the
  device reports the new version).

  The MQTT user needs write access to presence/+/cmd/ota/manifest. The
  password is read from $env:LD2420_OTA_MQTT_PASSWORD, else from
  -SecretFile (default ~/.esp-keys/ld2420_ota_mqtt_password.txt), else
  prompted for.

.EXAMPLE
  ./tools/ota_release.ps1 -Notes "Fix zone flicker"
  ./tools/ota_release.ps1 -Clean
#>
[CmdletBinding()]
param(
    [string]$HaSsh = "haos",
    [string]$HaUrl = "http://192.168.1.62:8123",
    [string]$DeviceId = "presence-bacad4",
    [string]$MqttUser = "ota_release",
    [string]$Notes = "",
    [string]$SecretFile = (Join-Path $env:USERPROFILE ".esp-keys\ld2420_ota_mqtt_password.txt"),
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot
$Bin = Join-Path $Root "build\presence_sensor_ld2420.bin"
$PubKey = Join-Path $PSScriptRoot "ota_signing_pubkey.pem"
$RemoteBase = "/config/www/ota/$DeviceId"
$Topic = "presence/$DeviceId/cmd/ota/manifest"

if ($DeviceId -notmatch '^[a-z0-9-]+$') { throw "Unexpected DeviceId '$DeviceId'" }
if ($MqttUser -notmatch '^[A-Za-z0-9_.-]+$') { throw "Unexpected MqttUser '$MqttUser'" }

function Get-MqttPassword {
    if ($env:LD2420_OTA_MQTT_PASSWORD) { return $env:LD2420_OTA_MQTT_PASSWORD }
    if (Test-Path $SecretFile) { return (Get-Content $SecretFile -Raw).Trim() }
    $secure = Read-Host "MQTT password for '$MqttUser'" -AsSecureString
    return [Net.NetworkCredential]::new("", $secure).Password
}

# Sends the password on the first stdin line so it never appears in a local
# command line; mosquitto_pub then reads the message (-s) or sends none (-n).
function Publish-Manifest([string]$Message) {
    $password = Get-MqttPassword
    $mode = if ($Message) { "-s" } else { "-n" }
    $remote = "IFS= read -r P; mosquitto_pub -h core-mosquitto -p 1883 -u '$MqttUser' -P `"`$P`" -t '$Topic' -q 1 -r $mode"
    "$password`n$Message" | ssh $HaSsh $remote
    if ($LASTEXITCODE -ne 0) { throw "mosquitto_pub failed (check the '$MqttUser' login and ACL)" }
}

if ($Clean) {
    ssh $HaSsh "rm -rf '$RemoteBase'"
    if ($LASTEXITCODE -ne 0) { throw "Failed to remove $RemoteBase" }
    Publish-Manifest ""
    Write-Host "Removed hosted firmware and cleared the retained manifest."
    return
}

if (-not (Test-Path $Bin)) { throw "No build found at $Bin - run idf.py build first." }

# Version the image actually carries (esp_app_desc_t.version at offset 0x30)
# must match PROJECT_VER, which is what the device checks after download.
$bytes = [IO.File]::ReadAllBytes($Bin)
$imageVersion = [Text.Encoding]::ASCII.GetString($bytes, 0x30, 32).TrimEnd([char]0)
$cmake = Get-Content (Join-Path $Root "CMakeLists.txt") -Raw
if ($cmake -notmatch 'set\(PROJECT_VER "([^"]+)"\)') { throw "PROJECT_VER not found in CMakeLists.txt" }
$projectVersion = $Matches[1]
if ($imageVersion -ne $projectVersion) {
    throw "Build is version '$imageVersion' but PROJECT_VER is '$projectVersion' - rebuild first."
}

$python = if ($env:IDF_PYTHON_ENV_PATH) { Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\python.exe" } else { "python" }
& $python -m espsecure verify_signature --version 2 --keyfile $PubKey $Bin *> $null
if ($LASTEXITCODE -ne 0) {
    throw "Image is not signed with the OTA key; the device would reject it."
}

$sha256 = (Get-FileHash -Algorithm SHA256 $Bin).Hash.ToLowerInvariant()
$size = (Get-Item $Bin).Length
$token = -join ([Security.Cryptography.RandomNumberGenerator]::GetBytes(16) | ForEach-Object { $_.ToString("x2") })
$remoteDir = "$RemoteBase/$token"
$url = "$HaUrl/local/ota/$DeviceId/$token/firmware.bin"

Write-Host "Publishing $projectVersion ($size bytes, sha256 $sha256)"

ssh $HaSsh "rm -rf '$RemoteBase' && mkdir -p '$remoteDir'"
if ($LASTEXITCODE -ne 0) { throw "Failed to prepare $remoteDir" }
scp -q $Bin "${HaSsh}:$remoteDir/firmware.bin"
if ($LASTEXITCODE -ne 0) { throw "scp failed" }

$remoteSha = (ssh $HaSsh "sha256sum '$remoteDir/firmware.bin'") -split '\s+' | Select-Object -First 1
if ($remoteSha -ne $sha256) { throw "Uploaded file hash mismatch ($remoteSha)" }

$head = Invoke-WebRequest -Uri $url -Method Head -SkipHttpErrorCheck
if ($head.StatusCode -ne 200) { throw "HA does not serve $url (HTTP $($head.StatusCode))" }

$manifest = [ordered]@{
    version = $projectVersion
    url     = $url
    sha256  = $sha256
    size    = $size
    notes   = $Notes
} | ConvertTo-Json -Compress

Publish-Manifest $manifest
Write-Host "Manifest published. Open the device in HA and press Install on 'Firmware'."
Write-Host "After the device reports $projectVersion, run: ./tools/ota_release.ps1 -Clean"
