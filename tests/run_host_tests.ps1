$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot
$OutDir = Join-Path $Root "build\host-tests"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$MqttExe = Join-Path $OutDir "ha_mqtt_host_test.exe"
$Ld2420Exe = Join-Path $OutDir "ld2420_host_test.exe"

# ha_mqtt parses the OTA manifest with cJSON; build it from the IDF tree.
$IdfPath = if ($env:IDF_PATH) { $env:IDF_PATH } else { "C:\esp\v5.5.3\esp-idf" }
$CJsonDir = Join-Path $IdfPath "components\json\cJSON"

gcc `
  -std=c11 `
  -Wall `
  -Wextra `
  -Werror `
  -I "$Root\tests\fakes" `
  -I "$Root\components\ha_mqtt\include" `
  -I "$CJsonDir" `
  "$Root\tests\ha_mqtt_host_test.c" `
  "$CJsonDir\cJSON.c" `
  -o $MqttExe

if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}

& $MqttExe
if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}

gcc `
  -std=c11 `
  -Wall `
  -Wextra `
  -Werror `
  -I "$Root\tests\fakes" `
  -I "$Root\components\ld2420\include" `
  "$Root\tests\ld2420_host_test.c" `
  -o $Ld2420Exe

if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}

& $Ld2420Exe
if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}
