$ErrorActionPreference = 'Stop'

$env:IDF_PATH = 'D:\esp32\.espressif\v5.4.4\esp-idf'
$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v5.4.4\venv'
$env:PATH = 'C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\git\cmd;C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;' + $env:PATH

$py = 'C:\Espressif\tools\python\v5.4.4\venv\Scripts\python.exe'
$idf = Join-Path $env:IDF_PATH 'tools\idf.py'
$proj = Join-Path $PSScriptRoot 'esp32_voice'

Set-Location $proj
Remove-Item build, sdkconfig -Recurse -Force -ErrorAction SilentlyContinue

& $py $idf set-target esp32s3
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $py $idf build
if (Test-Path (Join-Path $proj 'build\pid_voice.bin')) { exit 0 }
exit $LASTEXITCODE
