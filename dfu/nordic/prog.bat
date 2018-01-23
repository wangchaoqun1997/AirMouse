@echo off

set JLINK=36410082
set LOCK_SD_FROM_READBACK=1

set BL_SETTING_VALID_APP_ADDR=0x0007f000
set BL_SETTING_APP_CRC_ADDR=0x0007f01c
set BL_SETTING_APP_SIZE_ADDR=0x0007f020

if "%LOCK_SD_FROM_READBACK%"=="0" set DFU_OPT=--dfu

if "%1" == "" goto end
if "%2" == "" goto end
echo *** To program softdevice and bootloader ***
nrfjprog.exe -s %JLINK% -e  
nrfjprog.exe -s %JLINK%  --program %1 
nrfjprog.exe -s %JLINK%  %DFU_OPT% --program %2
echo.

echo *** To program application ***
nrfjprog.exe  -s %JLINK% --program %3


echo.

echo *** To reset target ***
nrfjprog.exe  -s %JLINK% -r

:end

