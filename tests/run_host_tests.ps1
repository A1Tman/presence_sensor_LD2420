$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot
$OutDir = Join-Path $Root "build\host-tests"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$MqttExe = Join-Path $OutDir "ha_mqtt_host_test.exe"
$Ld2420Exe = Join-Path $OutDir "ld2420_host_test.exe"

gcc `
  -std=c11 `
  -Wall `
  -Wextra `
  -Werror `
  -I "$Root\tests\fakes" `
  -I "$Root\components\ha_mqtt\include" `
  "$Root\tests\ha_mqtt_host_test.c" `
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
