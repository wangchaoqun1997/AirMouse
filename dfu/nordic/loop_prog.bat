@echo off

set SD_HEX_PATH="X:\nRF5_SDK_13.1.0_7ca7556\dfu\nordic\shadow\s132_nrf52_4.0.2_softdevice.hex"
set BL_HEX_PATH="X:\nRF5_SDK_13.1.0_7ca7556\dfu\nordic\shadow\bootloader_and_settings.hex"
set APP_HEX_PATH="X:\nRF5_SDK_13.1.0_7ca7556\dfu\nordic\shadow\nrf52832_xxaa.hex"



if "%1"=="" (
	
	set start_loop=4
	
) else (

	set start_loop=%1
)

setlocal enableextensions enabledelayedexpansion

set /a loop_count=1 > NUL

:loop_start
if %loop_count% gtr %start_loop% goto loop_end
 
echo *** Please prepare the target to be programmed ***	

;pause

echo *** Start programming target %loop_count% ***

call prog.bat %SD_HEX_PATH% %BL_HEX_PATH% %APP_HEX_PATH% 

set /a loop_count+=1 > NUL

goto loop_start

:loop_end

endlocal


