X:\nRF5_SDK_13.1.0_7ca7556\dfu\tools\mergehex.exe --merge X:\nRF5_SDK_13.1.0_7ca7556\dfu\nrf52832_xxaa_s132_bootloader.hex X:\nRF5_SDK_13.1.0_7ca7556\dfu\settings.hex --output X:\nRF5_SDK_13.1.0_7ca7556\dfu\bootloader_and_settings.hex
X:\nRF5_SDK_13.1.0_7ca7556\dfu\tools\mergehex.exe --merge X:\nRF5_SDK_13.1.0_7ca7556\dfu\bootloader_and_settings.hex X:\nRF5_SDK_13.1.0_7ca7556\dfu\nrf52832_xxaa.hex X:\nRF5_SDK_13.1.0_7ca7556\dfu\s132_nrf52_4.0.2_softdevice.hex --output X:\nRF5_SDK_13.1.0_7ca7556\dfu\whole.hex

"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfjprog.exe" --eraseall -f NRF52
"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfjprog.exe" --program X:\nRF5_SDK_13.1.0_7ca7556\dfu\whole.hex --verify -f NRF52
"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfjprog.exe" -s 36410082 -r
pause

