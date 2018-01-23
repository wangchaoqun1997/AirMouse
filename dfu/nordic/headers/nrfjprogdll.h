#ifndef NRFJPROGDLL_FUNC_H
#define NRFJPROGDLL_FUNC_H


#include <stdint.h>
#include "DllCommonDefinitions.h"


#if defined(__cplusplus)
extern "C" {
#endif

/* Expected log function prototype for logging operations. */
typedef void msg_callback(const char * msg_str);


/**
 * @brief	Returns the JLinkARM.dll version.
 *
 * @param	major								Pointer for storing of dll major version.
 * @param	minor								Pointer for storing of dll minor version.
 * @param	revision							Pointer for storing of dll revision.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 * @retval	INVALID_PARAMETER					The major parameter is NULL.
 *												The minor parameter is NULL.
 *												The revision parameter is NULL.
 */
nrfjprogdll_err_t dll_version(uint32_t * major, uint32_t * minor, char * revision);


/**
 * @brief	Opens the JLinkARM DLL and sets the log callback. Prepares the dll for work with a specific nRF device family.
 *
 * @details	This function searches for the appropriate family sub dll and load its function
 *			pointers to redirect further commands. It will call the family sub dll open
 *			function with the received JLinkARM.DLL path. This JLinkARM.dll path should include
 *          the name of the DLL	itself (i.e. "JLinkARM.dll"). Only JLinkARM.dlls whose versions
 *          are greater than a minimum version will be accepted. The minimum version for the 
 *          JLinkARM.dll is defined in MIN_JLINK_MAJOR_VERSION and MIN_JLINK_MINOR_VERSION defines.
 *          The log callback may be NULL. In that case no logging mechanism is provided. The 
 *          msg_callback typedef is defined elsewhere in this file.
 *
 * @param	jlink_path								Path to the JLinkARM DLL.
 * @param	cb										Callback for reporting informational and error messages.
 * @param	family									Defines the device family the next commands are going to be called to.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION						The open_dll function has already been called.
 * @retval	INVALID_PARAMETER						The jlink_path is NULL.
 *													The provided device family is not supported by this DLL.
 * @retval  JLINKARM_DLL_TOO_OLD                    The version of JLinkARM is lower than the minimum version required.                                                  
 * @retval	JLINKARM_DLL_NOT_FOUND					The jlink_path did not yield a usable DLL.
 * @retval	JLINKARM_DLL_COULD_NOT_BE_OPENED		An error occurred while opening the JLinkARM DLL.
 * 													A required function could not be loaded from the DLL.
 * @retval  NRFJPROG_SUB_DLL_NOT_FOUND				The nrfjprog_sub_dll path did not yield a usable DLL.
 * @retval  NRFJPROG_SUB_DLL_COULD_NOT_BE_OPENED	An error occurred while opening the NRFJPROG SUB DLL.
 *													A required function could not be loaded from the DLL.
 */
nrfjprogdll_err_t open_dll(const char * jlink_path, msg_callback * cb, device_family_t family);


/**
 * @brief	Closes and frees the JLinkARM DLL.
 *
 * @details	This function needs to be called before exiting if open_dll has been called.
 */
void close_dll(void);


/**
 * @brief	Enumerates the serial numbers of connected USB JLink emulators.
 *
 * @details	This function asks the JLink driver how many USB JLink emulators are present
 *			and writes that value to the num_available parameter. It then copies
 *			up to to the serial_numbers_len limit numbers serial numbers into the serial_numbers
 *			array. It is acceptable to call this function with serial_numbers set to NULL
 *			and serial_numbers_len set to zero in order to just obtain the number of
 *			connected emulators.
 *
 * @param	serial_numbers						Array in which to store the enumerated serial numbers.
 * @param	serial_numbers_len					Number of uint32_t values that can be stored in the
 *												serial_numbers array (may be zero).
 * @param	num_available						The number of serial numbers that were enumerated.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	INVALID_PARAMETER					The serial_numbers parameter is NULL but serial_numbers_len is > 0.
 *												The num_available parameter is NULL.
 * @retval	OUT_OF_MEMORY						Could not allocate buffer for reading serial numbers.
 */
nrfjprogdll_err_t enum_emu_snr(uint32_t serial_numbers[], uint32_t serial_numbers_len, uint32_t * num_available);


/**
 * @brief	Checks if the emulator has an established connexion with Segger emulator/debugger.
 *
 * @param	is_pc_connected_to_emu				Pointer of location to store the result.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 * @retval	INVALID_PARAMETER					The is_connected_to_emu pointer is NULL.
 */
nrfjprogdll_err_t is_connected_to_emu(bool * is_pc_connected_to_emu);


/**
 * @brief	Connects to a given emulator/debugger.
 *
 * @details	This function connects to serial_number emulator and sets the SWD communication speed at
 *          clock_speed_in_khz.
 *
 * @param	serial_number						Serial number of the emulator to connect to.
 * @param	clock_speed_in_khz					Speed for the SWD communication. Must be between JLINKARM_SWD_MIN_SPEED_KHZ
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has already been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	LOW_VOLTAGE							Low voltage was detected at the target device.
 * @retval	INVALID_PARAMETER					The clock_speed_in_khz parameter is not within limits.
 * @retval	EMULATOR_NOT_CONNECTED				The serial_number emulator is not connected to the PC. 
 */
nrfjprogdll_err_t connect_to_emu_with_snr(uint32_t serial_number, uint32_t clock_speed_in_khz);


/**
 * @brief	Connects to an emulator/debugger.
 *
 * @details	This function connects to an available emulator and sets the SWD communication speed at
 *          clock_speed_in_khz. If more than one emulator is available, a pop-up window will 
 *			appear to make a selection.
 *
 * @param	clock_speed_in_khz					Speed for the SWD communication. Must be between JLINKARM_SWD_MIN_SPEED_KHZ
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has already been called.
 * @retval	NO_EMULATOR_CONNECTED				There is no emulator connected to the PC.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	LOW_VOLTAGE							Low voltage was detected at the target device.										
 * @retval	INVALID_PARAMETER					The clock_speed_in_khz parameter is not within limits.
 */
nrfjprogdll_err_t connect_to_emu_without_snr(uint32_t clock_speed_in_khz);


/**
 * @brief	Disconnects from an emulator.
 *
 * @details	This function disconnects from a connected emulator. This also disconnects from a connected device if connected. Will
 *			not fail if we have never connected to an emulator.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 */
nrfjprogdll_err_t disconnect_from_emu(void);


/**
 * @brief	Recovers the device.
 *
 * @details	This operation attempts to recover the device and leave it as it was when it left Nordic factory.  It will attempt to 
 *          connect, erase all user available flash, halt and eliminate any protection. After recover, all RAM will be turned on.
 *          Note that this operation may take up to 30s for NRF51 devices.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval  NVMC_ERROR							Flash operation failed.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t recover(void);


/**
 * @brief	Checks if the emulator has an established connexion with nRF device.
 *
 * @param	is_emu_connected_to_device			Pointer of location to store the result.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The is_emu_connected_to_device pointer is NULL.
 */
nrfjprogdll_err_t is_connected_to_device(bool * is_emu_connected_to_device);


/**
 * @brief	Connects to the nRF device and halts it.
 *
 * @details	This function connects to the nRF device connected to the connected emulator/debugger and halts it.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 *												The connect_to_device has already been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t connect_to_device(void);


/**
 * @brief	Protects the device against read or debug.
 *
 * @details	Protects the device against read or debug by writing into UICR.RBPCONF and resetting. The function 
 *          will attempt to connect to the device if disconnected. The function will disconnect with a pin reset
 *          after execution. 
 *
 * @param	desired_protection					Desired protection level of read-back protect.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The desired_protection is REGION_0 or BOTH. [NRF52 only]
 *      										The desired_protection is NONE.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 * @retval  NVMC_ERROR							Flash operation failed.
 */
nrfjprogdll_err_t readback_protect(readback_protection_status_t desired_protection);


/**
 * @brief	Returns the status of the read-back protection.
 *
 * @details	Returns the status of the read-back protection. For NRF51, if emu is not connected
 *          to device, connect_to_device() will be called. For NRF51, if the device is not halted, 
 *          halt() will be called.
 *
 * @param	status								Pointer for storing of read-back status.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The status pointer is NULL.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device. [NRF51 only]
 */
nrfjprogdll_err_t readback_status(readback_protection_status_t * status);


/**
 * @brief	Returns the region 0 size and source of protection if any.
 *
 * @details	Returns the region 0 size and source of protection if any. For NRF51, if emu is not connected
 *          to device, connect_to_device() will be called. For NRF51, if the device is not halted, 
 *          halt() will be called. For NRF52, returns 0 size and NO_REGION_0 source since region 0 cannot
 *			be configured in this family.
 *
 * @param	size								Pointer for storing of region 0 protection size.
 * @param	source								Pointer for storing of region 0 protection source.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called. [NRF51 only]
 * @retval	INVALID_PARAMETER					The size pointer is NULL.
 *      	                 					The source pointer is NULL.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error. [NRF51 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device. [NRF51 only]
 */
nrfjprogdll_err_t read_region_0_size_and_source(uint32_t * size, region_0_source_t * source);


/**
 * @brief	Executes a system reset request.
 *
 * @details	Halts the core and executes a cortex M standard sys_reset_request. Note that after this operation the core will be halted.
 *          If emu is not connected to device, connect_to_device() will be called. If nRF CPU is not halted, halt() will be called.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t sys_reset(void);


/**
 * @brief	Executes a pin reset.
 *
 * @details	For NRF52, executes a pin reset by lowering to GND the nReset pin in the SWD connector for 20ms. For NRF51,
 *          executes a pin reset by toggling SWIO and SWCLK lines as explained in NRF51 user manual. For NRF51, to avoid
 *          problems with emulated SystemOff, a system_reset() is executed prior to the pin_reset(). Will not be possible
 *          if the device is an XLR or XLR2 and is read-back protected by PALL. Note that after this operation the core
 *          will be running.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device. [NRF51 only]
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The device is one of the early versions and is read-back protected by PALL. [NRF51 only]
 */
nrfjprogdll_err_t pin_reset(void);


/**
 * @brief	Disables BPROT.
 *
 * @details	Disables BPROT by writing into register BPROT.DISABLEINDEBUG and if NRF51 a sys_reset(). If emu is not 
 *          connected to device, connect_to_device() will be called. If nRF CPU is not halted, halt() will be called.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t disable_bprot(void);


/**
 * @brief	Erases all flash.
 *
 * @details	Erases all code and UICR flash regions. If NRF51, it will only work if the device is not programmed at the Nordic
 *          factory with a SoftDevice. If emu is not connected to device, connect_to_device() will be called. If nRF CPU
 *          is not halted, halt() will be called. Note that erase_all() will not disable BPROT if enabled while debug.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval  NVMC_ERROR							Flash operation failed.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t erase_all(void);


/**
 * @brief	Erases a page of code flash.
 *
 * @details	Erases a page of code flash beginning at the addr. If NRF51, it will only work if the page is not protected in region 0,
 *          either at Nordic factory or by the user. If emu is not connected to device, connect_to_device() will be called.
 *          If nRF CPU is not halted, halt() will be called. Note that erase_page() will not disable BPROT if enabled while debug.
 *
 * @param	addr								Address of the code flash page to erase.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval  NVMC_ERROR							Flash operation failed.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t erase_page(uint32_t addr);


/**
 * @brief	Erases UICR.
 *
 * @details	Erases UICR info page. If NRF51, it will only work if the device is programmed at the Nordic factory with a 
 *          SoftDevice. If emu is not connected to device, connect_to_device() will be called. If nRF CPU is not halted,
 *          halt() will be called.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval  NVMC_ERROR							Flash operation failed.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t erase_uicr(void);


/**
 * @brief	Writes one uint32_t data at the given address.
 *
 * @details	Writes one uint32_t data to the given addr without verifying that the address is accessible or even existing.
 *          If nvmc_control is true, it will control the NVMC in order to write into flash. Writes need to be 32bit aligned.
 *          Note that if the target address is in RAM that is unpowered, the operation will fail. If emu is not connected
 *          to device, connect_to_device() will be called. If nRF CPU is not halted, halt() will be called.
 *
 * @param	addr								Address to write to.
 * @param	data								Value to write.
 * @param	nvmc_control						If the target address needs NVMC control.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The addr parameter is not 32-bits aligned.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 *                                              The address to write is in unpowered RAM.
 * @retval  NVMC_ERROR							Flash operation failed.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t write_u32(uint32_t addr, uint32_t data, bool nvmc_control);


/**
 * @brief	Reads one uint32_t address.
 *
 * @details	Reads one uint32_t data from the given addr without verifying that the address is accessible or even existing.
 *          Reads need to be 32bit aligned. Note that if the target address is in RAM that is unpowered, the operation will fail.
 *          If emu is not connected to device, connect_to_device() will be called. If nRF CPU is not halted, halt() will be called.
 *
 * @param	addr								Address to read from.
 * @param	data								Pointer of location to store the value.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The addr parameter is not 32-bits aligned.
 *												The data parameter is null.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 *                                              The address to read is in unpowered RAM.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t read_u32(uint32_t addr, uint32_t * data);


/**
 * @brief	Writes data from the array starting at the given address.
 *
 * @details	Writes data_len bytes from the data array to the given address without verifying that the address is accessible
 *          or even existing. If nvmc_control is true, it will control the NVMC in order to write into flash. Writes need
 *          to be 32bit aligned. Writes	need to be in 4 byte aligned lengths. Note that if the target address is in RAM that
 *          is unpowered, the operation will fail. If emu is not connected to device, connect_to_device() will be called.
 *          If nRF CPU is not halted, halt() will be called.
 *
 * @param	addr								Start address of region to write to.
 * @param	data								Pointer to an array with the data to write.
 * @param	data_len							Length of data array.
 * @param	nvmc_control						If the target address needs NVMC control.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The addr parameter is not 32-bits aligned.
 *												The data_len parameter is 0.
 *												The data_len parameter is not a multiple of 4.
 *												The data parameter is NULL.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 *                                              The address to write is in unpowered RAM.
 * @retval  NVMC_ERROR							Flash operation failed.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t write(uint32_t addr, const uint8_t * data, uint32_t data_len, bool nvmc_control);


/**
 * @brief	Reads data_len bytes starting at address addr.
 *
 * @details	Reads data_len bytes starting at the given addr without verifying that the addresses are accessible or even 
 *          existing. Note that if the target address is in RAM that is unpowered, the operation will fail. If emu is not
 *          connected to device, connect_to_device() will be called. If nRF CPU is not halted, halt() will be called.
 *
 * @param	addr								Address to read from.
 * @param	data								Pointer of location to store the value.
 * @param	data_len							Number of bytes to read.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The data parameter is null.
 *												The data_len parameter is 0.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 *                                              The address to write is in unpowered RAM.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t read(uint32_t addr, uint8_t * data, uint32_t data_len);


/**
 * @brief	Checks if the nRF CPU is halted.
 *
 * @details Checks if device is halted.
 *          If emu is not connected to device, connect_to_device() will be called.
 *
 * @param	is_device_halted					Pointer of location to store the result.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	INVALID_PARAMETER					The is_device_halted pointer is NULL.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t is_halted(bool * is_device_halted);


/**
 * @brief	Halts the nRF CPU.
 *
 * @details Halts the nRF CPU.
 *          If emu is not connected to device, connect_to_device() will be called.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t halt(void);


/**
 * @brief	Starts the nRF CPU with the given pc and sp.
 *
 * @details Starts the nRF CPU with the given pc and sp.
 *          If emu is not connected to device, connect_to_device() will be called.
 *          If nRF CPU is not halted, halt() will be called.
 *
 * @param	pc									Program Counter to start running from.
 * @param	sp									Stack Pointer to use when running.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t run(uint32_t pc, uint32_t sp);


/**
 * @brief	Starts the nRF CPU.
 *
 * @details Starts the nRF CPU.
 *          If emu is not connected to device, connect_to_device() will be called.
 *          If nRF CPU is not halted, halt() will be called.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t go(void);


/**
 * @brief	Reads the RAM power status.
 *
 * @details Reads the RAM power status. Will read the status of the ram sections and return if they are either on or off.
 *          Parameter ram_sections_number is used by the dll to indicate how many of the ram_sections_power_status are 
 *          used to store results. Parameter ram_sections_size is used to report the size of those RAM sections. As a special
 *          use case, if ram_sections_power_status_array_size is 0, the dll will report the size and number of ram power
 *          sections, but will not populate ram_sections_power_status array. If emu is not connected to device, connect_to_device()
 *          will be called. If nRF CPU is not halted, halt() will be called. For NRF52, the operation will fail if the device
 *          is protected by PALL.
 *
 * @param	ram_sections_power_status				Array to store the results.
 * @param   ram_sections_power_status_array_size	Size of ram_sections_power_status array.
 * @param   ram_sections_number						Pointer of location to store the number of RAM sections in the device.
 * @param   ram_sections_size						Pointer of location to store the size of RAM sections in the device.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The ram_sections_power_status pointer is NULL and ram_sections_power_status_array_size is different than 0.
 *												The ram_sections_size pointer is NULL.
 *												The ram_sections_number pointer is NULL.
 *                                              The ram_sections_power_status_array_size is not 0 but is less than the number of ram power sections in the device, 4 for NRF51 XLR3, 2 for all other NRF51 devices and 16 for NRF52 FP1.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t is_ram_powered(ram_section_power_status_t * ram_sections_power_status, uint32_t ram_sections_power_status_array_size, uint32_t * ram_sections_number, uint32_t * ram_sections_size);


/**
 * @brief	Powers up all RAM sections of the device.
 *
 * @details Powers up all the RAM of the device in ON state. Will not affect the RAM retention in systemOff. If emu is not connected to device,
 *			connect_to_device() will be called. If nRF CPU is not halted, halt() will be called.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t power_ram_all(void);


/**
 * @brief	Powers down a RAM section of the device.
 *
 * @details Powers down a RAM section of the device in ON state. Will not affect the RAM retention in SystemOff. If emu is not connected to device,
 *			connect_to_device() will be called. If nRF CPU is not halted, halt() will be called. Will fail if the device is protected by PALL.
 *
 * @param	section_index			            Section of ram to power down.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The section section_index does not exist in the device.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection.
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t unpower_ram_section(uint32_t section_index);


/**
 * @brief	Reads a CPU register.
 *
 * @details	Reads a CPU register. Valid registers are R0-R15, XPSR, MSP and PSP.
 *          If emu is not connected to device, connect_to_device() will be called.
 *          If nRF CPU is not halted, halt() will be called.
 *
 * @param	register_name						Register name to read. See cpu_registers_t definition for valid values.
 * @param	register_value						Pointer of location to store the read register.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The register_data parameter is null.
												The register_name parameter is not a valid register.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t read_cpu_register(cpu_registers_t register_name, uint32_t * register_value);


/**
 * @brief	Writes a CPU register.
 *
 * @details	Writes a CPU register. Valid registers are R0-R15, XPSR, MSP and PSP.
 *          If emu is not connected to device, connect_to_device() will be called.
 *          If nRF CPU is not halted, halt() will be called.
 *
 * @param	register_name						CPU register to write. See cpu_registers_t definition for valid values.
 * @param	register_value						Value to write into the CPU register.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The register_name parameter is not a valid register.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t write_cpu_register(cpu_registers_t register_name, uint32_t register_value);


/**
 * @brief	Reads the device version connected to the device.
 *
 * @details	Reads the device version of the nRF device connected.
 *          If emu is not connected to device, connect_to_device() will be called.
 *          If nRF CPU is not halted, halt() will be called.
 *
 * @param	version								Pointer of location to store the device type.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The version parameter is null.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error.
 * @retval	NOT_AVAILABLE_BECAUSE_PROTECTION	The operation is not available due to read-back protection. [NRF52 only]
 * @retval	CANNOT_CONNECT						It is impossible to connect to any nRF device.
 */
nrfjprogdll_err_t read_device_version(device_version_t * version);


/**
 * @brief	Reads a debugger debug port register.
 *
 * @details	Reads into data pointer a debugger debug port register. 
 *
 * @param	reg_addr							Register address to read, either in debug port or access port.
 * @param	data								Pointer of location to store the value read.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The data parameter is null.
 *                                              The register address is not 32bit aligned.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error. 
 */
nrfjprogdll_err_t read_debug_port_register(uint8_t reg_addr, uint32_t * data);


/**
 * @brief	Writes a debugger debug port register.
 *
 * @details	Writes data parameter into a debugger debug port register.
 *
 * @param	reg_addr							Register address to write, either in debug port or access port.
 * @param	data								Data to write into the register.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The data parameter is null.
 *                                              The register address is not 32bit aligned.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error. 
 */
nrfjprogdll_err_t write_debug_port_register(uint8_t reg_addr, uint32_t data);


/**
 * @brief	Reads a debugger access port register.
 *
 * @details	Reads into data pointer a debugger access port register.
 *
 * @param	ap_index							Access port index for read if ap_access.
 * @param	reg_addr							Register address to read, either in debug port or access port.
 * @param	data								Pointer of location to store the value read.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The data parameter is null.
 *                                              The register address is not 32bit aligned.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error. 
 */
nrfjprogdll_err_t read_access_port_register(uint8_t ap_index, uint8_t reg_addr, uint32_t * data);


/**
 * @brief	Writes a debugger access port register.
 *
 * @details	Writes data parameter into a debugger access port register.
 *
 * @param	ap_index							Access port index for write if ap_access.
 * @param	reg_addr							Register address to write, either in debug port or access port.
 * @param	data								Data to write into the register.
 *
 * @retval	SUCCESS
 * @retval	INVALID_OPERATION					The open_dll function has not been called.
 *												The jlinkarm_connect_with_snr or jlinkarm_connect_without_snr has not been called.
 * @retval	INVALID_PARAMETER					The data parameter is null.
 *                                              The register address is not 32bit aligned.
 * @retval	JLINKARM_DLL_ERROR					JLinkARM dll function returned an error. 
 */
nrfjprogdll_err_t write_access_port_register(uint8_t ap_index, uint8_t reg_addr, uint32_t data);

#if defined(__cplusplus)
}
#endif

#endif  /* NRFJPROGDLL_FUNC_H */
