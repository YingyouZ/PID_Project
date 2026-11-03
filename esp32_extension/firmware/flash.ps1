param([Parameter(Mandatory=$true)][string]$Port)

$env:IDF_PATH = 'D:\esp32\.espressif\v5.4.4\esp-idf'
$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v5.4.4\venv'
$py = 'C:\Espressif\tools\python\v5.4.4\venv\Scripts\python.exe'

Set-Location $PSScriptRoot
& $py -m esptool --chip esp32s3 -p $Port -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB `
  0x0 bootloader/bootloader.bin `
  0x8000 partition_table/partition-table.bin `
  0x10000 pid_voice.bin `
  0x310000 srmodels/srmodels.bin