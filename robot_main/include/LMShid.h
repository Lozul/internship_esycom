// this file contains the device specific equates for the Vaunix LMS synthesizer
//
// RSD 12-2006 Version for Vaunix synthesizer
// 
// RSD 3-2007 Version for Dual VCO
// RSD 7-2008 Version for External Reference Oscillator
// RSD 4-2010 New commands for the DSS Synthesizer
// RSD 8-4-10 Moved RF_ON status bit to 0x10 position so there is room for the bidirectional
//            sweep status bit at 0x08
//
// RSD 10-2010 Version for use in the LMS DLL project.
// HME 02-20-2010 Copied from DLL project code for the Linux user version
// HME 02-28-2011 Changed pulse parameters to float
// RSD 6-2015 	Updated LMSPARAMS to include LMSType, PowerScale
//		PowerScale is 1 for .25db native units, 2 for .5db native units, 4 for 1db native units, etc.
//		PowerScale does not necessarily indicate the actual HW power level resolution, just the encoding used in the
//		command to the HW
//		Pulse Modulation functions changed/implemented to match the Windows API
// RSD 11-2015 Updated to include support for the external sweep option
// HME 09-27-2016 Added in some error codes that normally live in errno-base.h
//-----------------------------------------------------------------------------


// Format of the status report, for the convenience of host side coders

typedef struct 
{
  char pkt_status;
  char count;
  long frequency;
  char dev_status;
  signed char power;
} VNX_STATUS_REPORT;

#define STATUS_PLL_LOCK 0x80 		// MASK: Pll lock status bit, 1 = locked
#define STATUS_NEW_PARAM 0x40 		// MASK: A parameter was set since the last "Save Settings" command
#define STATUS_CMD_OK 0x20 		// MASK: The command was completed by the LMS device without problems
					// !!! Note !!! renamed but value unchanged to avoid a name collision
					// with the status return code for the DLL itself.
#define STATUS_RF_ON 0x10 		// MASK: The RF HW is on (moved from 0x08)


// Bit masks and equates for the Sweep command byte (stored in Sweep_mode, and reported also in Status)
#define SWP_BIDIR 0x08			// MASK: bit = 0 for ramp style sweep, 1 for triangle style sweep
#define SWP_DIRECTION 0x04		// MASK: bit = 0 for sweep up, 1 for sweep down 
#define SWP_CONTINUOUS 0x02		// MASK: bit = 1 for continuous sweeping
#define SWP_ONCE 0x01			// MASK: bit = 1 for single sweep


// Bit masks and equates for the DSS Pulse mode command byte (stored in Pulse_mode)
#define PM_ACTIVE 0x01 			// MASK: bit = 0 for normal mode, 1 for pulse mode
#define PM_EXTERNAL 0x02 		// MASK: bit = 0 for normal mode, 1 for external modulation

// HID report equates
#define HR_BLOCKSIZE 6 			// size of the block of bytes buffer in our HID report
#define HID_REPORT_LENGTH 8 		// use an 8 byte report..


typedef struct 
{
  char reportid;			// an artifact of the Windows HID API
  char status;
  char count;
  char byteblock[HR_BLOCKSIZE];
} HID_REPORT1;



typedef struct 
{
  char reportid;			// an artifact of the Windows HID API
  char command;
  char count;
  char byteblock[HR_BLOCKSIZE];
} HID_REPORT_OUT;


// Misc. constants used for the Vaunix devices
#define VNX_MIN_SWEEPTIME 1 		// 1 ms. minimum sweep time (AO checked 10-25)
#define MAX_ATTEN 64 			// we've got 64 db max attenuation for most LMS devices 
					// we represent it as 4 * the value, or the attenuation in .25db steps at the API level
#define MAX_ATTEN_LH 80 		// we've got 80 db max attenuation for LMS-6123LH devices 
					// we represent it as 4 * the value, or the attenuation in .25db steps at the API level
								
								
// Misc commands to send to the device
// For the input reports the first byte is the status, for the output it is the command. The high bit sets the 
// direction.
//
//count is the number of valid bytes in the byteblock array
// byteblock is an array of bytes which make up the value of the command's argument or arguments.
//
// For result reports the command portion of the status byte is equal to the command sent if the command was successful.
// status byte layout:

// Bit0 - Bit5 = command result, equal to command if everything went well
// Bit6 = --reserved--
// Bit7 = --reserved--

// All frequency related data items are DWORD (unsigned) quantities, stored in normal Microsoft byte order.
// Sweep time is a DWORD (unsigned)
// Pulse mod times use the top two bits to encode their range


// Misc commands to send to the device

#define VNX_SET 0x80
#define VNX_GET 0x00 		// the set and get bits are or'd into the msb of the command byte

// ---- LSG Commands, some used in other devices ----
#define VNX_FREQUENCY 0x04 	// output frequency in 100Khz units
#define VNX_FDWELL 0x05 	// time to dwell at each frequency in the sweep, in 1ms units, minimum of 10ms.
#define VNX_FSTART 0x06 	// frequency lower limit of sweep in 100Khz units 
#define VNX_FSTOP 0x07 		// frequency upper limit of sweep in 100Khz units
#define VNX_FSTEP 0x08 		// frequency step size, in 100Khz units
#define VNX_SWEEP 0x09 		// command to start/stop sweep, data = 01 for single sweep, 00 to stop
				// sweeping, and 02 for continuous sweeping.
				// For the DSS synth, there are more sweep option bits

#define VNX_RFMUTE 0x0A 	// enable or disable RF output, byte = 01 to enable, 00 to disable
#define VNX_ATTEN 0x0B 		// attenuator setting, byte = number of dbm attenuation in .25 db steps
				// 00 is no attenuation, 02 is .5 db, 04 is 1 db

#define VNX_SAVEPAR 0x0C 	// command to save user parameters to flash, data bytes must be
				// set to  0x42, 0x55, 0x31 as a key to enable the flash update
				// all of the above settings are saved (Frequency, sweep params, etc.)

#define VNX_PWR 0x0D 		// power output setting, relative to calibrated value - adds to calibrated
				// attenuator setting. It is a byte, with the attenuation expressed in .25db
				// steps. Thus, 2 = .5 db attenuation, 4 = 1.0db attenuation, etc. 

#define VNX_DEFAULTS 0x0F 	// restore all settings to factory default
				// FSTART = Minimum Frequency, FSTOP = Maximum Frequency

#define VNX_STATUS 0x0E 	// Not really a command, but the status byte value for periodic status reports.

#define VNX_INTOSC 0x23 	// enable/disable internal reference oscillator - Byte == 01 for on, 00 for off

// ---------------------- DSS Specific Commands ----------------------

#define VNX_DSS_STATUS 0x4E 	// Not really a command, but the status byte value for periodic status reports.

#define VNX_DSS_FREQUENCY 0x44 	// output frequency in 10 Hz units
#define VNX_DSS_SWPTIME 0x45 	// sweep time, in 1ms units, minimum of 100ms.
#define VNX_DSS_FSTART 0x46 	// frequency lower limit of sweep in 10 Hz units 
#define VNX_DSS_FSTOP 0x47 	// frequency upper limit of sweep in 10 Hz units
#define VNX_DSS_SWEEP 0x48 	// start/stop sweeps

#define VNX_OFFTIME 0x49 	// length of pulse mode RF off time in microseconds
#define VNX_ONTIME 0x4A 	// length of pulse mode RF on time in microseconds
#define VNX_PULSE_MODE 0x4B 	// start/stop pulse mode

#define VNX_EXTSWP 0x4C		// enable/disable external sweep trigger

// ---------------------- Misc. commands ---------------------------
#define VNX_MAX_PWR 0x1B 	// get the calibrated power output
#define VNX_GETSERNUM 0x1F	// get the serial number, value is a DWORD
#define VNX_MINFREQUENCY 0x20 	// get (no set allowed) the minimum frequency
#define VNX_MAXFREQUENCY 0x21 	// get (no set allowed) the maximum frequency

#define VNX_MODELNAME 0x22 	// get (no set allowed) the device's model name string -- last 6 chars only



// ----------- Global Equates ------------
#define MAXDEVICES 64
#define MAX_MODELNAME 32

#define PMODE_1US 0x00		// the top nibble is 0 for pulse mode times in 1 microsecond units
#define PMODE_1MS 0x20		// the top nibble is 2 for pulse mode times in 1 millisecond units
#define PTIME_MASK 0x00FFFFFF	// only the first 24 bits are used for pulse mode times

// ----------- Data Types ----------------

#define DEVID unsigned int

typedef struct
{
  int DevStatus;
  int Frequency;
  int MinFrequency;
  int MaxFrequency;
  int StartFrequency;
  int EndFrequency;
  int FrequencyStep;
  int SweepTime;
  int Power;
  int MaxPower;
  int MinPower;
  int PowerScale;
  int PulseOnTime;
  int PulseOffTime;
  float PulseOnTimeSeconds;
  float PulseOffTimeSeconds;
  int Modebits;
  int SerialNumber;
  char ModelName[MAX_MODELNAME];
  int LMSType;
  /* so we can find this device again later */
  unsigned int idVendor;
  unsigned int idProduct;
  char Serialstr[16];
  char thread_command;
  char outbuff[8];
  char pending;
  int MyDevID;

} LMSPARAMS;

// ----------- Mode Bit Masks ------------

#define MODE_RFON 0x00000010 		// bit is 1 for RF on, 0 if RF is off
#define MODE_INTREF 0x00000020 		// bit is 1 for internal osc., 0 for external reference
#define MODE_SWEEP 0x0000000F 		// bottom 4 bits are used to keep the sweep control bits

#define MODE_PWMON 0x00000100 		// we keep a copy of the PWM control bits here, 1 for int PWM on
#define MODE_EXTPWM 0x00000200 		// 1 for ext. PWM input enabled
#define PWM_MASK 0x00000300

#define MODE_EXTSWEEP 0x00001000	// bit is 1 for external sweep trigger, 0 for internal sweep control 

// ----------- Command Equates -----------


// Status returns for commands
#define LVSTATUS unsigned int

#define STATUS_OK 			0
#define BAD_PARAMETER  		0x80010000 	// out of range input -- frequency outside min/max etc.
#define BAD_HID_IO     		0x80020000	// an error was returned by the lower level USB drivers
#define DEVICE_NOT_READY  	0x80030000 	// device isn't open, no handle, etc.
#define ERROR_STATUS_MSK	0x80000000	// the MSB is set for errors

#define F_INVALID_DEVID		-1.0		// for functions that return a float, STATUS_OK is the same
#define F_BAD_HID_IO		-2.0
#define F_DEVICE_NOT_READY	-3.0

// Status returns for DevStatus
#define INVALID_DEVID 		0x80000000 	// MSB is set if the device ID is invalid
#define DEV_CONNECTED 		0x00000001 	// LSB is set if a device is connected
#define DEV_OPENED 		0x00000002 	// set if the device is opened
#define SWP_ACTIVE 		0x00000004 	// set if the device is sweeping
#define SWP_UP 			0x00000008 	// set if the device is sweeping up in frequency
#define SWP_REPEAT 		0x00000010 	// set if the device is in continuous sweep mode
#define SWP_BIDIRECTIONAL 	0x00000020 	// set if the device is in bi-directional sweep mode
#define PLL_LOCKED 		0x00000040	// set if the PLL lock status is TRUE (both PLL's are locked)
#define	FAST_PULSE_OPTION 	0x00000080	// set if the fast pulse mode option is installed
#define EXT_SWEEP_OPTION	0x00000100	// set if the external sweep option is enabled (HW may not be installed)

// Internal values in DevStatus
#define DEV_LOCKED 		0x00002000 	// set if we don't want read thread updates of the device parameters
#define DEV_RDTHREAD 		0x00004000 	// set when the read thread is running

// Flags to encode pulse mode time ranges
#define PM48Mhz			0x10000000	// used to select the 48Mhz pulse mod clock
#define PM1Mhz			0x00000000	// used to select the 1Mhz pulse mod clock or sw pulsing

#ifdef __cplusplus
extern "C"
{
#endif
    void fnLMS_Init(void);
    void fnLMS_SetTestMode(bool testmode);
    int fnLMS_GetNumDevices();
    int fnLMS_GetDevInfo(DEVID *ActiveDevices);
    int fnLMS_GetModelName(DEVID deviceID, char *ModelName);
    int fnLMS_InitDevice(DEVID deviceID);
    int fnLMS_CloseDevice(DEVID deviceID);
    int fnLMS_GetSerialNumber(DEVID deviceID);

    LVSTATUS fnLMS_SetFrequency(DEVID deviceID, int frequency);
    LVSTATUS fnLMS_SetStartFrequency(DEVID deviceID, int startfrequency);
    LVSTATUS fnLMS_SetEndFrequency(DEVID deviceID, int endfrequency);
    LVSTATUS fnLMS_SetSweepTime(DEVID deviceID, int sweeptime);

    LVSTATUS fnLMS_SetPowerLevel(DEVID deviceID, int powerlevel);
    LVSTATUS fnLMS_SetRFOn(DEVID deviceID, bool on);

    // *** Pulse mode functions were changed in V1.01 to match the Windows API ***
    //LVSTATUS fnLMS_SetPulseOnTime(DEVID deviceID, int pulseontime);
    //LVSTATUS fnLMS_SetPulseOffTime(DEVID deviceID, int pulseofftime);
    //LVSTATUS fnLMS_SetFastPulsedOutput(DEVID deviceID, int pulseontime, int pulsereptime, bool on);
    LVSTATUS fnLMS_SetPulseOnTime(DEVID deviceID, float pulseontime);
    LVSTATUS fnLMS_SetPulseOffTime(DEVID deviceID, float pulseofftime);
    // SetPulseOnTime and SetPulseOffTime require applications to ensure that correct operational ranges are selected
    // Use of SetFastPulsedOutput is recommended
    LVSTATUS fnLMS_SetFastPulsedOutput(DEVID deviceID, float pulseontime, float pulsereptime, bool on);
    LVSTATUS fnLMS_EnableInternalPulseMod(DEVID deviceID, bool on);
    LVSTATUS fnLMS_SetUseExternalPulseMod(DEVID deviceID, bool external);

    LVSTATUS fnLMS_SetUseInternalRef(DEVID deviceID, bool internal);
    LVSTATUS fnLMS_SetUseExtSweepTrigger(DEVID deviceID, bool external);
    LVSTATUS fnLMS_SetSweepDirection(DEVID deviceID, bool up);
    LVSTATUS fnLMS_SetSweepMode(DEVID deviceID, bool mode);
    LVSTATUS fnLMS_SetSweepType(DEVID deviceID, bool swptype);
    LVSTATUS fnLMS_StartSweep(DEVID deviceID, bool go);
    LVSTATUS fnLMS_SaveSettings(DEVID deviceID);

    int fnLMS_GetFrequency(DEVID deviceID);
    int fnLMS_GetStartFrequency(DEVID deviceID);
    int fnLMS_GetEndFrequency(DEVID deviceID);
    int fnLMS_GetSweepTime(DEVID deviceID);

    int fnLMS_GetRF_On(DEVID deviceID);
    int fnLMS_GetUseInternalRef(DEVID deviceID);
    int fnLMS_GetUseExtSweepTrigger(DEVID deviceID);
    int fnLMS_GetPowerLevel(DEVID deviceID);
    int fnLMS_GetMaxPwr(DEVID deviceID);
    int fnLMS_GetMinPwr(DEVID deviceID);
    int fnLMS_GetMaxFreq(DEVID deviceID);
    int fnLMS_GetMinFreq(DEVID deviceID);

    // *** Pulse mode functions were changed in V1.01 to match the Windows API ***
    //int fnLMS_GetPulseOnTime(DEVID deviceID);
    //int fnLMS_GetPulseOffTime(DEVID deviceID);
    float fnLMS_GetPulseOnTime(DEVID deviceID);
    float fnLMS_GetPulseOffTime(DEVID deviceID);
    int fnLMS_GetPulseMode(DEVID deviceID);
    int fnLMS_GetHasFastPulseMode(DEVID deviceID);
    int fnLMS_GetUseInternalPulseMod(DEVID deviceID);

    char* fnLMS_perror(LVSTATUS status);
    char* fnLMS_LibVersion(void);
#ifdef __cplusplus
}
#endif

/* We use libusb and it returns codes that normally live in errno-base.h. Since
   that isn't always linkable in all systems, we'll put the codes in here.

   These codes come from libusb 1.0.9. If a different version is used, the error
   codes could change. */

#ifndef EPERM
#define	EPERM		 1	/* Operation not permitted */
#endif

#ifndef ENOENT
#define	ENOENT		 2	/* No such file or directory */
#endif

#ifndef ESRCH
#define	ESRCH		 3	/* No such process */
#endif

#ifndef EINTR
#define	EINTR		 4	/* Interrupted system call */
#endif

#ifndef EIO
#define	EIO		 5	/* I/O error */
#endif

#ifndef ENXIO
#define	ENXIO		 6	/* No such device or address */
#endif

#ifndef E2BIG
#define	E2BIG		 7	/* Argument list too long */
#endif

#ifndef ENOEXEC
#define	ENOEXEC		 8	/* Exec format error */
#endif

#ifndef EBADF
#define	EBADF		 9	/* Bad file number */
#endif

#ifndef ECHILD
#define	ECHILD		10	/* No child processes */
#endif

#ifndef EAGAIN
#define	EAGAIN		11	/* Try again */
#endif

#ifndef ENOMEM
#define	ENOMEM		12	/* Out of memory */
#endif

#ifndef EACCES
#define	EACCES		13	/* Permission denied */
#endif

#ifndef EFAULT
#define	EFAULT		14	/* Bad address */
#endif

#ifndef ENOTBLK
#define	ENOTBLK		15	/* Block device required */
#endif

#ifndef EBUSY
#define	EBUSY		16	/* Device or resource busy */
#endif

#ifndef EEXIST
#define	EEXIST		17	/* File exists */
#endif

#ifndef EXDEV
#define	EXDEV		18	/* Cross-device link */
#endif

#ifndef ENODEV
#define	ENODEV		19	/* No such device */
#endif

#ifndef ENOTDIR
#define	ENOTDIR		20	/* Not a directory */
#endif

#ifndef EISDIR
#define	EISDIR		21	/* Is a directory */
#endif

#ifndef EINVAL
#define	EINVAL		22	/* Invalid argument */
#endif

#ifndef ENFILE
#define	ENFILE		23	/* File table overflow */
#endif

#ifndef EMFILE
#define	EMFILE		24	/* Too many open files */
#endif

#ifndef ENOTTY
#define	ENOTTY		25	/* Not a typewriter */
#endif

#ifndef ETXTBSY
#define	ETXTBSY		26	/* Text file busy */
#endif

#ifndef EFBIG
#define	EFBIG		27	/* File too large */
#endif

#ifndef ENOSPC
#define	ENOSPC		28	/* No space left on device */
#endif

#ifndef ESPIPE
#define	ESPIPE		29	/* Illegal seek */
#endif

#ifndef EROFS
#define	EROFS		30	/* Read-only file system */
#endif

#ifndef EMLINK
#define	EMLINK		31	/* Too many links */
#endif

#ifndef EPIPE
#define	EPIPE		32	/* Broken pipe */
#endif

#ifndef EDOM
#define	EDOM		33	/* Math argument out of domain of func */
#endif

#ifndef ERANGE
#define	ERANGE		34	/* Math result not representable */
#endif

