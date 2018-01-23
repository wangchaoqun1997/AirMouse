
"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfjprog.exe"  --eraseall -f NRF52
"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfjprog.exe"  --program whole.hex --verify -f NRF52 
"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfjprog.exe"  --memwr 0x0007F000 --val 0x01 --verify -f NRF52          
"C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin\nrfjprog.exe"  --debugreset -f NRF52

pause

