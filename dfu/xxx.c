
Usage: 
-------------------------------------------------------------------------------

     --licenses              Displays the licenses of the open source modules
                             used in nrfjprog.exe.

 -q  --quiet                 Reduces the stdout info. Must be combined with
                             another command.

 -h  --help                  Displays this help.

 -v  --version               Displays the nrfjprog and dll versions.

     --jdll <file>           Uses the Segger's JLinkARM dll specified in the
                             given file path instead of searching for the
                             latest version of Segger's JLinkARM dll. Must be
                             combined with another command.

     --qspiini <file>        Uses the QSPI settings file specified in the given
                             file path instead of searching for the default
                             settings file. Must be combined with either
                             --memrd, --memwr, --program, --verify, --erasepage
                             or --qspieraseall commands.
                             Limitations:
                             For nRF51 devices, the operation is not available.
                             For nRF52 devices, the operation is not available
                             for devices without a QSPI peripheral.

     --qspicustominit        Use the QSPI custom instructions defined in the
                             QSPI settings file used in the operation when
                             initializing the QSPI peripheral. For an example
                             of how to specify the custom instruction to
                             execute, see the QSPIDefault.ini file. Must be
                             combined with either --memrd, --memwr, --program,
                             --verify, --erasepage or --qspieraseall commands.
                             Limitations:
                             For nRF51 devices, the operation is not available.
                             For nRF52 devices, the operation is not available
                             for devices without a QSPI peripheral.

 -i  --ids                   Displays the serial numbers of all the debuggers
                             connected to the PC.

 -f  --family <family>       Selects the device family for the operation. Valid
                             argument options are NRF51, NRF52 and UNKNOWN. If
                             UNKNOWN family is given, an automatic family
                             detection of the device is performed. Note that
                             providing the actual family is faster than
                             performing the automatic family detection. If
                             --family option is not given, the default is taken
                             from nrfjprog.ini. Must be combined with another
                             command.

 -s  --snr <serial_number>   Selects the debugger with the given serial number
                             among all those connected to the PC for the
                             operation. Must be combined with another command.

 -c  --clockspeed <speed>    Sets the debugger SWD clock speed in kHz
                             resolution for the operation. The valid clockspeed
                             arguments go from 125 kHz to 50000 kHz. If given
                             clockspeed is above the maxiumum clockspeed
                             supported by the emulator, its maximum will be
                             used instead. If --clockspeed option is not given,
                             the default is taken from nrfjprog.ini. Must be
                             combined with another command.

     --recover               Erases all user available non-volatile memory and
                             disables the read back protection mechanism if
                             enabled.

     --rbp <level>           Enables the readback protection mechanism. Valid
                             argument options are CR0 and ALL.
                             Limitations:
                             For nRF52 devices, the CR0 argument option is
                             invalid.
                             Side effects:
                             After an --rbp operation is performed, the
                             available operations are reduced.
                             For nRF51 devices, and if argument option ALL is
                             used, --pinreset will not work on certain older
                             devices.
                             For nRF52 devices, only --pinreset, --debugreset
                             or --recover operations are available after --rbp.

     --pinresetenable        Enables the pin reset by the use of UICR PSELRESET
                             registers.
                             Limitations:
                             For nRF51 devices, the command is not available.

 -p  --pinreset              Performs a pin reset. Core will run after the
                             operation.

 -r  --reset                 Performs a soft reset by setting the SysResetReq
                             bit of the AIRCR register of the core. The core
                             will run after the operation. Can be combined with
                             the --program operation. If combined with the
                             --program operation, the reset will occur after
                             the flashing has occurred to start execution.

 -d  --debugreset            Performs a soft reset by the use of the CTRL-AP.
                             The core will run after the operation. Can be
                             combined with the --program operation. If combined
                             with the --program operation, the debug reset will
                             occur after the flashing has occurred to start
                             execution.
                             Limitations:
                             For nRF51 devices, the --debugreset operation is
                             not available.
                             For nRF52 devices, the --debugreset operation is
                             not available for nRF52832_xxAA_ENGA devices.

 -e  --eraseall              Erases all user available program flash memory and
                             the UICR page. Can be combined with the
                             --qspieraseall operation.
                             Limitations:
                             For nRF51 devices, if the device came from Nordic
                             with a pre-programmed SoftDevice, only the user
                             available code flash and UICR will be erased.

     --qspieraseall          Erases all the flash of the external memory device
                             with the help of the QSPI peripheral. Note that
                             depending on the external memory device's erase
                             speed the operation might take minutes. Can be
                             combined with the --eraseall operation.
                             Limitations:
                             For nRF51 devices, the operation is not available.
                             For nRF52 devices, the operation is not available
                             for devices without a QSPI peripheral.
                             For nRF52 devices, the operation is only available
                             for devices connected to an external memory
                             device. To determine if an external memory device
                             is connected, nrfjprog checks MemSize parameter
                             from the QspiDefault.ini file or the QSPI
                             configuration ini file that is given with the
                             --qspiini option.

     --eraseuicr             Erases the UICR page.
                             Limitations:
                             For nRF51 devices, the operation is available only
                             if the device came from Nordic with a
                             pre-programmed SoftDevice.

     --erasepage <start[-end]>                      Erases the flash pages
                             starting at start address and ending at end
                             address (not included in the erase). If end
                             address is not given, only one flash page will be
                             erased. If your device is equipped with a QSPI
                             peripheral, the pages to erase belong to the XIP
                             region of the device, and an external memory
                             device is present, this command erases 4 kB pages
                             from the external memory device. The first address
                             of the region is considered as address 0 of the
                             external memory device. To determine if an
                             external memory device is present, nrfjprog checks
                             the MemSize parameter from the QspiDefault.ini
                             file or from the QSPI configuration ini file given
                             with the --qspiini option.
                             Limitations:
                             For nRF51 devices, the page will not be erased if
                             it belongs to region 0.

     --program <hex_file> [--sectorerase | --chiperase | --sectoranduicrerase]
                             [--qspisectorerase | --qspichiperase]
                             Programs the specified hex_file into the device.
                             If the target area to program is not erased, the
                             --program operation will fail unless an erase
                             option is given. Valid erase operations for
                             the internal flash memory are --sectorerase,
                             --sectoranduicrerase and --chiperase. If
                             --chiperase is given, all the available user
                             non-volatile memory, including UICR, will be
                             erased before programming. If --sectorerase is
                             given, only the targeted non-volatile memory pages
                             excluding UICR will be erased. If
                             --sectoranduicrerase is given, only the targeted
                             non-volatile memory pages including UICR will be
                             erased. Note that the --sectoranduicrerase and
                             --sectorerase operations normally take
                             significantly longer time compared to --chiperase
                             operation so use them with caution. If your
                             device is equipped with a QSPI peripheral and an
                             external memory device is present, data targeting
                             the XIP region will be written to the external
                             memory device. The first address of the XIP region
                             is considered as address 0 of the external memory
                             device. To determine if an external memory device
                             is present, nrfjprog checks the MemSize paramter
                             from QspiDefault.ini file or from the QSPI
                             configuration ini file given with the --qspiini
                             option. If the target area to program is not
                             erased, the --program operation will fail unless
                             an erase option is given. Valid erase operations
                             for the external memory device are --qspichiperase
                             and --qspisectorerase. If --qspichiperase is
                             given, the external memory device will be erased.
                             If the --qspisectorerase is given, only 4kB pages
                             pages from the targeted external memory device
                             will be erased. Note that --qspichiperase
                             operation may take several minutes. The --program
                             command can be combined with the --verify
                             operation. It can also be combined with either the
                             --reset or the --debugreset operations. the reset
                             will occur after the flash operation in order to
                             start the execution.
                             Limitations:
                             For nRF51 devices, the --sectoranduicrerase
                             operation is not available.
                             For nRF51 devices, if the hex_file provided
                             contains sectors belonging to region 0, the
                             --program operation will fail.
                             For nRF51 devices, the --qspisectorerase and
                             --qspichiperase operations are not available.
                             For nRF52 devices, the --qspisectorerase and
                             --qspichiperase operations are not available
                             unless the device is equipped with a QSPI
                             peripheral and an external memory is connected. To
                             determine if an external memory is present, the
                             MemSize parameter from QspiDefault.ini file or
                             from the QSPI configuration ini file given with
                             the --qspiini option is evaluated.

     --memwr <addr> --val <val>                     Writes to the provided
                             address in memory with the help of the NVM
                             Controller or, if your device is equipped with a
                             QSPI peripheral and the address to write belongs
                             to the XIP region, with the help of the QSPI
                             peripheral to an external memory device. To
                             determine if an external memory device is present,
                             nrfjprog checks the MemSize parameter from
                             QspiDefault.ini file or the QSPI configuration ini
                             file that is given with the --qspiini option.
                             The first address of the region is considered as
                             address 0 of the external memory device. If the
                             target address is flash (either internal or in the
                             external memory device) and not erased, the
                             operation will fail. This command can be combined
                             with the --verify operation.

     --ramwr <addr> --val <val>                     Writes to memory without
                             the help of the NVM Controller to the provided
                             address. If the target address is in non-volatile
                             memory, --ramwr will have no effect unless the
                             non-volatile memory controller (NVMC) has been
                             previously configured by the user for a write
                             operation. Can be combined with the --verify
                             operation.

     --verify [<hex_file>] [--fast]                 The provided hex_file
                             contents are compared with the contents in the
                             device code flash, RAM, UICR and XIP regions for
                             devices equipped with a QSPI peripheral connected
                             to an external memory device, and fail if there is
                             a mismatch. To determine if an external memory
                             device is present, nrfjprog checks MemSize
                             parameter from QspiDefault.ini file or from the
                             QSPI configuration ini file given with the
                             --qspiini option. If the optional --fast parameter
                             is given, nrfjprog will calculate a hash of the
                             flash target area using a sha256 algorithm and
                             compare it to the expected hash instead of
                             reading back the actual contents of the device's
                             flash in order to speed the operation. It can be
                             combined with the --program, --memwr and --ramwr
                             operations if provided without the hex_file
                             parameter.
                             Limitations:
                             For nRF51 devices, the --fast verifying option
                             is not available.

     --memrd <addr> [--w <width>] [--n <n>]         Reads n bytes from the
                             provided address. If width is not given, 32-bit
                             words will be read if addr is word aligned,
                             16-bit words if addr is half word aligned, and
                             8-bit words otherwise. If n is not given, one
                             word of size width will be read. The address and
                             n must be aligned to the width parameter. The
                             maximum number of bytes that can be read is 1 MB.
                             The width must be 8, 16 or 32. If your device is
                             equipped with a QSPI peripheral, and the addresses
                             to read belong to the XIP region, the QSPI
                             peripheral is used to read from the external
                             memory device if present. To determine if an
                             external memory device is present, mrfjprog checks
                             the MemSize parameter from QspiDefault.ini file or
                             the QSPI configuration ini file that is given with
                             the --qspiini option. The first address of the
                             region is considered as address 0 of the external
                             memory device.
                             Limitations:
                             A single --memrd instruction cannot be used to
                             read addresses from both the external memory
                             device and the nRF device.

     --halt                  Halts the CPU core.

     --run [--pc <pc_addr> --sp <sp_addr>]          Starts the CPU. If --pc and
                             --sp options are given, the pc_addr and sp_addr
                             are used as initial PC and stack pointer. For
                             pc_addr to be valid its last bit must be one. For
                             sp_addr to be valid it must be word aligned.

     --readuicr <file>       Reads the device UICR and stores it in the given
                             file name. Can be combined with --readram,
                             --readcode and --readqspi. If combined, only one
                             instruction can provide a file name.

     --readcode <file>       Reads the device flash and stores it in the given
                             file name. Can be combined with --readuicr,
                             --readram and --readqspi. If combined, only one
                             instruction can provide a file name.

     --readram <file>        Reads the device ram and stores it in the given
                             file name. Can be combined with --readuicr,
                             --readcode and --readqspi. If combined, only one
                             instruction can provide a file name.

     --readqspi <file>       Reads the QSPI-connected external memory and
                             stores it in the given file name. Can be combined
                             with --readuicr, --readcode and --readram. If
                             combined, only one instruction can provide a file
                             name.

     --readregs              Reads the cpu registers.

