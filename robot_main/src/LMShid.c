// -------------------------- Vaunix LMS Lab Brick Support Library -----------------------
//
// Copyright Vaunix 2011-2015
// You may use this software to develop applications that use the Vaunix LMS Synthesizers
// ---------------------------------------------------------------------------------------

#include <usb.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "LMShid.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define DEBUG_OUT 0  /* set this to 1 in order to see debugging output, 2 for a ton of output, or 3 for many tons */
#define FALSE 0
#define TRUE !FALSE

#define PACKET_CTRL_LEN 8
#define PACKET_INT_LEN 8
#define INTERFACE 0
#define ENDPOINT_INT_IN 0x82
#define TIMEOUT 500

/* RD 6/2015	updated to support new LMS models and slightly simplify model selection code
		fixed defaults for LMS-203
		changed/updated pulse mode functions to match the Windows API
		fixed code to ensure clearing of pending flag after each SendReport
   RD 8/2015	fixed power scaling and reporting code, it now returns absolute power level
		changed timing in GetParameter to allow the read thread to run during successive calls to GetParameter
   RD 11/2015	added support for external sweep option
   HME 09/2016  add in support for 4 new LMS models (sVNX16-19 in the table)
                fixed problem where disconnecting the device isn't detected correctly
*/
#define LIBVER "1.03"

void *brick_handler_function (void *ptr);

/* ----------------------------------------------------------------- */
// globals we'll be using at runtime
char errmsg[32]; 		// For the status->string converter

bool bVerbose = FALSE; 		// True to generate debug oriented printf output

bool TestMode = FALSE; 		// if TestMode is true we fake it -- no HW access
				// TestMode defaults to FALSE for production builds

LMSPARAMS lms [MAXDEVICES];	// an array of structures each of which holds the info for a given 
				// device. The DeviceID is the index into the array. Devices may come and go
				// so there is no guarantee that the active elements are contiguous

char lmspath [MAXDEVICES*MAX_MODELNAME]; // space to store the device path for each of our devices

char sVNX1[32] = "LMS-103";
char sVNX2[32] = "LMS-123";
char sVNX3[32] = "LMS-203";
char sVNX4[32] = "LMS-802";
char sVNX5[32] = "LMS-163";
char sVNX6[32] = "LMS-322D";
char sVNX7[32] = "LMS-232D";
char sVNX8[32] = "LMS-602D";
char sVNX9[32] = "LMS-402D";
char sVNX10[32] = "LMS-451D";
char sVNX11[32] = "LMS-271D";
char sVNX12[32] = "LMS-152D";
char sVNX13[32] = "LMS-751D";
char sVNX14[32] = "LMS-252D";
char sVNX15[32] = "LMS-6123LH";
char sVNX16[32] = "LMS-163LH";
char sVNX17[32] = "LMS-802LH";
char sVNX18[32] = "LMS-802DX";
char sVNX19[32] = "LMS-183DX";

unsigned short vnx_VID = 0x041f;  // device VID for Vaunix products

unsigned short dev1PID = 0x1220;  // device PID for Vaunix LMS-103
unsigned short dev2PID = 0x1222;  // device PID for Vaunix LMS-123
unsigned short dev3PID = 0x1223;  // device PID for Vaunix LMS-203
unsigned short dev4PID = 0x1221;  // device PID for Vaunix LMS-802
unsigned short dev5PID = 0x1224;  // device PID for Vaunix LMS-163 
unsigned short dev6PID = 0x1225;  // device PID for Vaunix LMS-322D
unsigned short dev7PID = 0x1226;  // device PID for Vaunix LMS-232D
unsigned short dev8PID = 0x1227;  // device PID for Vaunix LMS-602D
unsigned short dev9PID = 0x1228;  // device PID for Vaunix LMS-402D
unsigned short dev10PID = 0x1229; // device PID for Vaunix LMS-451D
unsigned short dev11PID = 0x122A; // device PID for Vaunix LMS-271D
unsigned short dev12PID = 0x122B; // device PID for Vaunix LMS-152D
unsigned short dev13PID = 0x122C; // device PID for Vaunix LMS-751D
unsigned short dev14PID = 0x122D; // device PID for Vaunix LMS-252D
unsigned short dev15PID = 0x122E; // device PID for Vaunix LMS-6123LH
unsigned short dev16PID = 0x122F; // device PID for Vaunix LMS-163LH
unsigned short dev17PID = 0x1250; // device PID for Vaunix LMS-802LH
unsigned short dev18PID = 0x1251; // device PID for Vaunix LMS-802DX
unsigned short dev19PID = 0x1252; // device PID for Vaunix LMS-183DX

/* stuff for the threads */
pthread_t threads[MAXDEVICES];
usb_dev_handle *thread_devhandles[MAXDEVICES];
#define THREAD_IDLE 0
#define THREAD_START 1
#define THREAD_WRITE 2
#define THREAD_EXIT 3
#define THREAD_DEAD 4
#define THREAD_ERROR -1

// --------------------- other device specific equates ------------------------
#define HW_MAXP_10 40 	// MaxPower is the output of the device in db -- +10 in this case
			// NB -- the value used in the SetPower function is relative attenuation
			// not absolute power!!
#define HW_MINP_40 160	// For LMS devices that have 40db output power range in .25dbunits)						
#define HW_MINP_50 200	// For LMS devices that have 50db output power range
#define HW_MINP_55 220	// For LMS devices that have 55db output power range
#define HW_MINP_80 320	// The LMS-6123LH has an 80db output power range


// --------------- Device IO support functions ----------------------------

bool CheckDeviceOpen(DEVID deviceID) {
  if (TestMode) return TRUE;		// in test mode all devices are always available
  if ((lms[deviceID].DevStatus & DEV_OPENED) && (deviceID != 0))
    return TRUE;
  else
    return FALSE;
}
// ------------------------------------------------------------------------
bool DevNotLocked(DEVID deviceID) {
  if (TestMode) return TRUE;	// this shouldn't happen, but just in case...
  if (!(lms[deviceID].DevStatus & DEV_LOCKED))
    return TRUE;				// we return TRUE if the device is not locked!
  else
    return FALSE;
}
// ------------------------------------------------------------------------
void LockDev(DEVID deviceID, bool lock) {
  if (TestMode) return;			// this shouldn't happen, but just in case...
  if (lock) {
    lms[deviceID].DevStatus = lms[deviceID].DevStatus | DEV_LOCKED;
    return;
  } else {
    lms[deviceID].DevStatus = lms[deviceID].DevStatus & ~DEV_LOCKED;
    return;
  }
}

// A function to display the status as a string
char* fnLMS_perror(LVSTATUS status) {
  strcpy(errmsg, "STATUS_OK");
  if (BAD_PARAMETER == status) strcpy(errmsg, "BAD_PARAMETER");
  if (BAD_HID_IO == status) strcpy(errmsg, "BAD_HID_IO");
  if (DEVICE_NOT_READY == status) strcpy(errmsg, "DEVICE_NOT_READY");
  
  // Status returns for DevStatus
  if (INVALID_DEVID == status) strcpy(errmsg, "INVALID_DEVID");
  if (DEV_CONNECTED == status) strcpy(errmsg, "DEV_CONNECTED");
  if (DEV_OPENED == status) strcpy(errmsg, "DEV_OPENED");
  if (SWP_ACTIVE == status) strcpy(errmsg,  "SWP_ACTIVE");
  if (SWP_UP == status) strcpy(errmsg, "SWP_UP");
  if (SWP_REPEAT == status) strcpy(errmsg, "SWP_REPEAT");
  if (SWP_BIDIRECTIONAL == status) strcpy(errmsg, "SWP_BIDIRECTIONAL");
  
  return errmsg;
}

// A function to display the status as a string for functions that return a floating point value
char* fnLMS_pFloatError(float fstatus) {
  strcpy(errmsg, "STATUS_OK");
  if (fstatus == F_INVALID_DEVID) strcpy(errmsg, "INVALID_DEVID");
  if (fstatus == F_BAD_HID_IO) strcpy(errmsg, "BAD_HID_IO");
  if (fstatus == F_DEVICE_NOT_READY) strcpy(errmsg, "DEVICE_NOT_READY");

  return errmsg;
}



// return the version of the library
char LibVersion[] = LIBVER;
char* fnLMS_LibVersion(void) {
  return LibVersion;
}
  

// functions based on hid_io.cpp
bool VNXOpenDevice(DEVID deviceID) {

  if (!(lms[deviceID].DevStatus & DEV_CONNECTED))	// we can't open a device that isn't there!
    return DEVICE_NOT_READY;
  
  if (DEBUG_OUT > 1) printf("Starting thread...\r\n");
  lms[deviceID].thread_command = THREAD_START; 		// open device and start processing
  pthread_create(&threads[deviceID], NULL, brick_handler_function, (void*)deviceID);
  lms[deviceID].DevStatus = lms[deviceID].DevStatus | DEV_OPENED;
  
  return STATUS_OK;
}

void report_data_decode(unsigned char rcvdata[], int tid) {
  int i;
  unsigned long dataval;
  char temp[32];

  if (DEBUG_OUT > 2) {
    printf("Decoding ");
    for (i=0; i<8; i++)
      printf("%02x ", rcvdata[i]);
    printf("\r\n");
  }

/*	the first byte tells us the type, the second is the data length
	tid is the thread ID it came from so we can stash the value into lms[]
	first decode the bytes
*/
  dataval = 0;
  if (0 < rcvdata[1]) {
    for (i=0; i<rcvdata[1]; i++)
      dataval = (dataval<<8) + rcvdata[1+rcvdata[1]-i];
  }
  if (DEBUG_OUT > 2) printf("Data payload decodes to %ld (%08x)\r\n", dataval, dataval);
  /* now we'll assign it to lms[] */

  // handle the status report
  switch(rcvdata[0]) {
  case VNX_DSS_STATUS:
    if (DevNotLocked(tid)) {
      lms[tid].Frequency = rcvdata[2] + (rcvdata[3]<<8) + (rcvdata[4]<<16) + (rcvdata[5]<<24); // update the frequency
      if (DEBUG_OUT > 2) printf("Status decode device = %d, Frequency = %d\r\n", tid, lms[tid].Frequency);
      if (DEBUG_OUT > 1) printf("."); fflush(0);

      // decode the status report and update our device status
      if (rcvdata[6] & (STATUS_PLL_LOCK)) 				// is the PLL locked?
	lms[tid].DevStatus = lms[tid].DevStatus | PLL_LOCKED ;
      else
	lms[tid].DevStatus = lms[tid].DevStatus & ~PLL_LOCKED;
	  
      if (rcvdata[6] & (SWP_ONCE | SWP_CONTINUOUS))			// are we sweeping?
	lms[tid].DevStatus = lms[tid].DevStatus | SWP_ACTIVE;
      else
	lms[tid].DevStatus = lms[tid].DevStatus & ~SWP_ACTIVE;

      if (rcvdata[6] & (SWP_DIRECTION))					// are we sweeping down in frequency?
	lms[tid].DevStatus = lms[tid].DevStatus & ~SWP_UP;		
      else
	lms[tid].DevStatus = lms[tid].DevStatus | SWP_UP;		// set the SWP_UP status bit if we are sweeping upwards

      if (rcvdata[6] & (SWP_CONTINUOUS))				// are we in continuous sweep mode?
	lms[tid].DevStatus = lms[tid].DevStatus | SWP_REPEAT;
      else
	lms[tid].DevStatus = lms[tid].DevStatus & ~SWP_REPEAT;

      if (rcvdata[6] & (SWP_BIDIR))					// are we in bi-directional sweep mode?
	lms[tid].DevStatus = lms[tid].DevStatus | SWP_BIDIRECTIONAL;
      else
	lms[tid].DevStatus = lms[tid].DevStatus & ~SWP_BIDIRECTIONAL;
	
    } /* if devnotlocked() */

    if (DEBUG_OUT > 2) printf("DevStatus at the end of ParseHidPacket: %d \r\n", (lms[tid].DevStatus & DEV_RDTHREAD));
    break;

  case VNX_DSS_FREQUENCY:
    if (DEBUG_OUT > 0) printf(" Frequency = %d\r\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].Frequency = dataval;
    break;

  case VNX_DSS_SWPTIME:
    if (DEBUG_OUT > 0) printf(" Sweep Time = %d\r\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].SweepTime = dataval;
    break;

  case VNX_DSS_FSTART:
    if (DEBUG_OUT > 0) printf(" Sweep Start Frequency = %d\r\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].StartFrequency = dataval;
    break;

  case VNX_DSS_FSTOP:
    if (DEBUG_OUT > 0) printf(" Sweep End Frequency = %d\r\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].EndFrequency = dataval;
    break;

  case VNX_SWEEP:
    if (DEBUG_OUT > 0) printf(" Sweep Mode = %d\r\n", rcvdata[2]);
    if (DevNotLocked(tid))
      lms[tid].Frequency = dataval;		//!!! RD not done yet !!!
    break;

  case VNX_ONTIME:
    if (DEBUG_OUT > 0) printf(" Pulse On Time = %d\r\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].PulseOnTime = dataval;
    break;

  case VNX_OFFTIME:
    if (DEBUG_OUT > 0) printf(" Pulse Off Time = %d\r\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].PulseOffTime = dataval;
    break;

  case VNX_PULSE_MODE:
    if (DEBUG_OUT > 0) printf(" Pulse Mode = %x\r\n", rcvdata[2]);
    dataval = rcvdata[2] & 0x03;		// make sure we only get two bits of mode
    if (DevNotLocked(tid)) {
	  if (rcvdata[2] & 0x80)		// the bit is true when the fast PWM option is not present
	  {
		lms[tid].DevStatus = lms[tid].DevStatus & ~FAST_PULSE_OPTION;
	  }
	  else					// the option is present
	  {
		lms[tid].DevStatus = lms[tid].DevStatus | FAST_PULSE_OPTION;
	  }
	  if (rcvdata[2] & 0x40)		// the bit is true when the fast PWM option _is_ present
	  {
		lms[tid].DevStatus = lms[tid].DevStatus | EXT_SWEEP_OPTION;
	  }
	  else					// the option is not present
	  {
		lms[tid].DevStatus = lms[tid].DevStatus & ~EXT_SWEEP_OPTION;
	  }
      lms[tid].Modebits = lms[tid].Modebits & ~PWM_MASK;	// clear the PWM bitfield
      dataval <<= 8;						// align the PWM bits with their bitfield
      lms[tid].Modebits = lms[tid].Modebits | dataval;		// or in the PWM mode bits
    }
    break;

  case VNX_RFMUTE:
    if (rcvdata[2]) strcpy(temp, "RF ON");
    else strcpy(temp, "RF OFF");
    if (DEBUG_OUT > 0) printf("%s \n", temp);

    if (DevNotLocked(tid)) {
      if (rcvdata[2]) {
		lms[tid].Modebits = lms[tid].Modebits | MODE_RFON;
      } else {
		lms[tid].Modebits = lms[tid].Modebits & ~MODE_RFON;
      }
    }
    break;

  case VNX_INTOSC:
    if (rcvdata[2]) strcpy(temp, "Using Internal Osc");
    else strcpy(temp, "Using External Osc");
    if (DEBUG_OUT > 0) printf("%s \r\n", temp);

    if (DevNotLocked(tid)) {
      if (rcvdata[2]) {
		lms[tid].Modebits = lms[tid].Modebits | MODE_INTREF;
      } else {
		lms[tid].Modebits = lms[tid].Modebits & ~MODE_INTREF;
      }
    }
    break;
    
  case VNX_EXTSWP:
    if (rcvdata[2]) strcpy(temp, "Using External Sweep Trigger");
    else strcpy(temp, "Using Internal Sweep Trigger");
    if (DEBUG_OUT > 0) printf("%s \r\n", temp);

    if (DevNotLocked(tid)) {
      if (rcvdata[2]) {
		lms[tid].Modebits = lms[tid].Modebits | MODE_EXTSWEEP;
      } else {
		lms[tid].Modebits = lms[tid].Modebits & ~MODE_EXTSWEEP;
      }
    }
    break;
    
    
  case VNX_PWR:
    if (DEBUG_OUT > 0) printf(" Raw Power Setting = %d units)\n", rcvdata[2]);
    if (DevNotLocked(tid))
      lms[tid].Power = lms[tid].MaxPower - (int)rcvdata[2] * lms[tid].PowerScale;	// API is always in .25db units, HW reports vary

    break;

  case VNX_MAX_PWR:
    if (DEBUG_OUT > 0) printf(" Max Power Setting = %d units)\n", rcvdata[2]);
    if (DevNotLocked(tid))
		lms[tid].MaxPower = (int)rcvdata[2];	// API and HW is in .25db units
    break;

  case VNX_MINFREQUENCY:
    if (DEBUG_OUT > 0) printf(" Minimum Frequency = %d\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].MinFrequency = dataval * 10000;	// our device returns MinFrequency in 100 KHz units
    break;

  case VNX_MAXFREQUENCY:
    if (DEBUG_OUT > 0) printf(" Maximum Frequency = %d\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].MaxFrequency = dataval * 10000;	// same as MinFrequency
    break;

  case VNX_GETSERNUM:
    if (DEBUG_OUT > 0) printf(" Serial Number = %d\n", dataval);
    if (DevNotLocked(tid))
      lms[tid].SerialNumber = dataval;		// NB -- we never use this path!
    break;
  } /* switch */

  return;
}

// ************* The read thread handler for the brick ***********************
void *brick_handler_function (void *threadID) {
  int i, tid;
  tid = (int)threadID;
  struct usb_bus *busses;
  struct usb_bus *bus;
  unsigned char sendbuff[8];
  unsigned char rcvbuff[32];
  int usb_status;
  char fullpath[128];
  int retries;
 
  if (DEBUG_OUT > 0) printf("Starting thread for device %d\r\n", tid);
  while ((lms[tid].thread_command >=0) &&
	 (lms[tid].thread_command != THREAD_EXIT)) {
    switch(lms[tid].thread_command) {
    case THREAD_IDLE: /* idle */
      /* this is where we wait for incoming USB data */
      //      printf("Starting a read...\r\n");
      /* If someone is writing to the device, don't try to read from it */
      if (lms[tid].pending) {
	if (DEBUG_OUT > 1) {printf("z"); fflush(0);}
	usleep(10000); /* wait 10ms for whatever to finish */
	goto TIDLE_DONE;
      }

      /* This is some code from the Windows library---
	if (RStatus == 0x48F)		// ERROR_DEVICE_NOT_CONNECTED
		{
			if (IO_Trace > 0)
				printf("Closing ReadReport loop due to HW loss \n");

			// Mark the device as no longer connected
			lms[MyDev].DevStatus = lms[MyDev].DevStatus & ~DEV_CONNECTED;

			// First free the ReadEvent object for this thread
			CloseHandle(lms[MyDev].ghReadEvent);
			lms[MyDev].ghReadEvent = NULL;

			// Now we'll try to  close the file handle to the device -- the hardware got up and left already!
			CloseHandle(lms[MyDev].hDevice);		// RD? -- has the OS already done the close on its own??
			lms[MyDev].hDevice = NULL;

			// Mark it closed in our list of devices
			lms[MyDev].DevStatus = lms[MyDev].DevStatus & ~DEV_OPENED;

			// And finally, we'll kill our own thread
			lms[MyDev].ghReadThread = NULL;			// clear our handle, we're about to kill ourselves
			ExitThread(0);

		}	// end of hot-unplug handler

      */

      usb_status = -1;
      retries = 50;
      while ((usb_status < 0) && (retries--) && (THREAD_IDLE == lms[tid].thread_command)) {
	usb_status = usb_interrupt_read(thread_devhandles[tid], // handle
					ENDPOINT_INT_IN,	// endpoint
					rcvbuff,		// buffer
					PACKET_INT_LEN, 	// max length
					TIMEOUT);
	if (usb_status < 0) usleep(1000); /* wait 20 ms before trying again */
      }

      if (usb_status >= 0) {
	if (DEBUG_OUT > 2) {
	  printf("Thread %d reports %d...", tid, usb_status);
	  for (i=0; i<usb_status; i++)
	    printf("%02x ", rcvbuff[i]);
	  printf("\r\n");
	}

	/* decode the HID data */
	report_data_decode(rcvbuff, tid);
      } else {
	if (DEBUG_OUT > 0) {
	  printf("Closing read thread loop due to HW loss\r\n");
	  perror("THREAD_IDLE");
	}
	/* Something bad happened. The only thing we can think of is that the device was disconnected. If that's
	   the case, we'll expect ENODEV which is error 19 in libusb 1.0.9. */
	if ((0-usb_status) == ENODEV) { /* remember that the table is positive... */
	  if (DEBUG_OUT > 0) printf("Device %d appears to have been disconnected\r\n", tid);
	  /* remove the device from the table of recognized devices */
	  // Mark the device as no longer connected
	  lms[tid].DevStatus = lms[tid].DevStatus & ~DEV_CONNECTED;

	  // Mark it closed in our list of devices
	  lms[tid].DevStatus = lms[tid].DevStatus & ~DEV_OPENED;

	  // And finally, we'll kill our own thread
	  lms[tid].thread_command = THREAD_EXIT;
	} else { /* some unexpected problem? */
	  if (DEBUG_OUT > 0) perror("THREAD_IDLE encountered unexpected error");
	} 
      }
    TIDLE_DONE:
      usleep(8000);  // 8 ms
      break;
    case THREAD_START: /* starting up */
      /* we'll open the device. First we have to locate it */
      if (DEBUG_OUT > 0) printf("Thread %d is looking for the device\r\n", tid);

      usb_find_busses();
      usb_find_devices();
      busses = usb_get_busses();

      lms[tid].thread_command = THREAD_ERROR; /* assume it will fail */
      for (bus = busses; bus; bus = bus->next) {
	if (THREAD_IDLE == lms[tid].thread_command) break;
	struct usb_device *dev;

	for (dev = bus->devices; dev; dev = dev->next) {
	  if (THREAD_IDLE == lms[tid].thread_command) break;
	  if (DEBUG_OUT > 1) printf("Thread %d sez- Vendor: %04x PID: %04x\r\n", tid, dev->descriptor.idVendor, dev->descriptor.idProduct);
	  thread_devhandles[tid] = usb_open(dev);
	  usb_status = usb_get_string_simple(thread_devhandles[tid], dev->descriptor.iSerialNumber, rcvbuff, sizeof(rcvbuff));
	  if (DEBUG_OUT > 1) printf("USB device opened, status: %d\r\n", usb_status);
	  if (DEBUG_OUT > 1) printf("string %d = [%s] looking to match [%s]\r\n", dev->descriptor.iSerialNumber, rcvbuff, lms[tid].Serialstr);
	  //	  usb_close(thread_devhandles[tid]);
	  if ((dev->descriptor.idVendor == lms[tid].idVendor) &&
	      (dev->descriptor.idProduct == lms[tid].idProduct) &&
	      (0 == strcmp(rcvbuff, lms[tid].Serialstr))) {
	    /* we found the device. We'll open it */
	    if (DEBUG_OUT > 1) printf("Opening file [%s]\r\n", dev->filename);
	    thread_devhandles[tid] = usb_open(dev);

	    usb_detach_kernel_driver_np(thread_devhandles[tid], 0);

	    usb_status = usb_set_configuration (thread_devhandles[tid], 1);
	    if (DEBUG_OUT > 1) printf ("set configuration: %s\n", usb_status ? "failed" : "passed");

	    usb_status = usb_claim_interface (thread_devhandles[tid], 0);
	    if (DEBUG_OUT > 1) printf ("claim interface: %s\n", usb_status ? "failed" : "passed");

	    lms[tid].thread_command = THREAD_IDLE;
	    break;
	  } //else
	  usb_close(thread_devhandles[tid]);
	  if (DEBUG_OUT > 2) printf("USB device closed\r\n");
	}
      }

       //     lms[tid].thread_command = THREAD_IDLE;
      break;
    case THREAD_WRITE:
#if 0 // we don't use the threaded write scheme...
      /* we have data to write to the device */
      printf("Thread %d: writing to handle %d ", tid, thread_filehandles[tid]);
      for (i=0; i<8; i++) {
	printf("%02x ", lms[tid].outbuff[i]);
      }
      //      usb_status = write(thread_filehandles[tid], lms[tid].outbuff, 8);
      //      printf("(status=%d)\r\n", usb_status);
      lms[tid].thread_command = THREAD_IDLE;
#endif
      break;
    } /* switch */
  } /* while */
  if (DEBUG_OUT > 0) printf("Exiting thread for device %d\r\n", tid);
  if (THREAD_EXIT == lms[tid].thread_command) {
    usb_close(thread_devhandles[tid]);
    thread_devhandles[tid] = 0;
  }
  lms[tid].thread_command = THREAD_DEAD;
  pthread_exit(NULL);
  
}

// -------------- SendReport -------------------------------------------------

bool SendReport(int deviceID, char command, char *pBuffer, int cbBuffer)
{
  int i;
  int send_status;
  int retries;
  // Make sure the buffer that is being passed to us fits
  if (cbBuffer > HR_BLOCKSIZE) {
    // Report too big, don't send!
    return FALSE;
  }

  char Report[8];
  
  if (DEBUG_OUT > 1) printf("SR: command=%x cbBuffer=%x\r\n", (command & 0xFF), cbBuffer);
  lms[deviceID].outbuff[0] = command;		// command to device
  lms[deviceID].outbuff[1] = cbBuffer;
  lms[deviceID].outbuff[2] = pBuffer[0];
  lms[deviceID].outbuff[3] = pBuffer[1];
  lms[deviceID].outbuff[4] = pBuffer[2];
  lms[deviceID].outbuff[5] = pBuffer[3];
  lms[deviceID].outbuff[6] = pBuffer[4];
  lms[deviceID].outbuff[7] = pBuffer[4];
  if (DEBUG_OUT > 2) {
    printf("SR: ");
    for (i=0; i<8; i++) {
      printf("%02x ", (lms[deviceID].outbuff[i] & 0xFF));
    }
    printf("\r\n");
  }

  /* we have to wait for a file handle to appear */
  retries = 0;
  while ((0 == thread_devhandles[deviceID]) && (retries++ < 10))
    sleep(1);
  /* we have data to write to the device */
  if (DEBUG_OUT > 2) printf("SR: sending the write...\r\n");
  send_status = usb_control_msg(thread_devhandles[deviceID],
				0x21,
				0x09, //HID_REPORT_SET,
				0x200,
				0,
				lms[deviceID].outbuff,
				PACKET_CTRL_LEN,
				TIMEOUT);

  if (DEBUG_OUT > 1) {
    printf("(status=%x handle=%x)", send_status, thread_devhandles[deviceID]);
    if (send_status < 0) perror("SendReport"); else printf("\r\n");
  }
  //lms[tid].thread_command = THREAD_IDLE;


#if 0
  lms[deviceID].thread_command = THREAD_WRITE;
#endif
  // We should have transferred exactly as many bytes as we sent
  //  assert(cbTransferred == sOutReportSize);
  
  //	printf(" sending command: %02x of %d bytes\n", command, cbTransferred );
  
  return TRUE;
}

// ------------ GetParameter ---------------------------------------------
//
// The GetParam argument is the command byte sent to the device to get
// a particular value. The response is picked up by the read thread and
// parsed by it. The parser clears the corresponding event.


bool GetParameter(int deviceID, int GetParam)
{
	char VNX_param[4] = {0, 0, 0, 0};

	lms[deviceID].pending = 1;
	usleep(10000); /* allow 10 ms for any read activity to stop */
	if (DEBUG_OUT > 0) printf(" sending a GET command = %x\n", (char) GetParam );
	if (!SendReport(deviceID, (char)GetParam, VNX_param, 0)) {
	  lms[deviceID].pending = 0;
	  usleep(80000);  // wait 80 ms so the read thread can get the response
	  return FALSE;
	}

	if (DEBUG_OUT > 0) printf(" SendReport sent a GET command successfully = %x\n", (char) GetParam );
	lms[deviceID].pending = 0;
	usleep(80000);  // wait 80 ms so the read thread can get the response
	return TRUE;
}

// -------------- Get Routines to read device settings --------------------
//
// Note: for these functions deviceID is not checked for validity
//		 since it was already checked in the calling program.

bool GetFrequency(DEVID deviceID) {
  if (DEBUG_OUT > 0) printf(" In GetFrequency\n");
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_DSS_FREQUENCY))
    return FALSE;
  return TRUE;
}

// -------------------------------

bool GetPower(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_PWR))
    return FALSE;
  return TRUE;
}

// -------------------------------

bool GetFStart(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_DSS_FSTART))
    return FALSE;
  return TRUE;
}

// -------------------------------

bool GetFEnd(DEVID deviceID) {	
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_DSS_FSTOP))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetSweep(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_DSS_SWPTIME))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetMaxPower(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_MAX_PWR))
    return FALSE;
  return TRUE;
}
// -------------------------------
bool GetRF_On(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_RFMUTE))
    return FALSE;
  return TRUE;
}

// ---------------------------------
bool GetUseIntOsc(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_INTOSC))
    return FALSE;
  return TRUE;
}

// ---------------------------------
bool GetUseExtSwpTrigger(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_EXTSWP))
    return FALSE;
  return TRUE;
}
bool GetFMinimum(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_MINFREQUENCY))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetFMaximum(DEVID deviceID) {
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_MAXFREQUENCY))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetPulseOnTime(DEVID deviceID) {	
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_ONTIME))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetPulseOffTime(DEVID deviceID) {	
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_OFFTIME))
    return FALSE;
  return TRUE;
}

// -------------------------------
bool GetPulseMode(DEVID deviceID) {	
  // --- first we send the command out to the device --
  if (!GetParameter(deviceID, VNX_PULSE_MODE))
    return FALSE;
  return TRUE;
}

/* functions to manage devices, not getting or retrieving data */
/*-------------------------------------------------------------*/

void FindVNXDevices()
{

  bool bFound;
  int hTemp;  		// temporary variable
  int HWType; 		// temporary variable for hardware/model type
  int HWMinFrequency; 	// temporary variable for default minimum frequency
  int HWMaxFrequency; 	// temporary variable for default maximum frequency
  int HWMaxPower; 	// temporary variable for default maximum power
  int HWMinPower;	// temporary variable for default minimum power
  int HWPowerScale;	// temporary variable for power control command scaling
  char HWName[32];  	// temporary variable for the hardware model name
  char HWSerial[8]; 	// temporary holder for the serial number
  struct usb_dev_handle *devhandle;
  struct usb_bus *busses;
  char sendbuff[8];
  char rcvbuff[32];
  int usb_status;
    
  usb_init();
  if (DEBUG_OUT > 2)
    usb_set_debug(3); 	// if we want lots of debug, let's see the USB output too.
  else
    usb_set_debug(0);
  usb_find_busses();
  usb_find_devices();
  
  busses = usb_get_busses();
        
  struct usb_bus *bus;
  int c, i, a;
  int send_status, open_status;
  
  /* ... */
    
  // We need to remove devices from our table that are no longer connected ---
  // to do this we clear the "connected" flag for each table entry that is not open initially
  // then, as we find them we re-set the "connected" flag
  // anybody who doesn't have his "connected" flag set at the end is gone - we found it
  // previously but not this time
  
  for (i = 1; i<MAXDEVICES; i++){
    if ((lms[i].SerialNumber != 0) && !(lms[i].DevStatus & DEV_OPENED))
      lms[i].DevStatus = lms[i].DevStatus & ~DEV_CONNECTED; 	
  }

  for (bus = busses; bus; bus = bus->next) {
    struct usb_device *dev;
    
    for (dev = bus->devices; dev; dev = dev->next) {
      if (DEBUG_OUT > 1) printf("Vendor: %04x PID: %04x\r\n", dev->descriptor.idVendor, dev->descriptor.idProduct);
      HWType = 0;
      /* check this device to see if it's one of our devices */
      if (dev->descriptor.idVendor == vnx_VID) {
		// and then see which one
	  	if (dev->descriptor.idProduct == dev1PID) HWType = 1;
		if (dev->descriptor.idProduct == dev2PID) HWType = 2;
		if (dev->descriptor.idProduct == dev3PID) HWType = 3;
		if (dev->descriptor.idProduct == dev4PID) HWType = 4;
		if (dev->descriptor.idProduct == dev5PID) HWType = 5;
		if (dev->descriptor.idProduct == dev6PID) HWType = 6;
		if (dev->descriptor.idProduct == dev7PID) HWType = 7;
		if (dev->descriptor.idProduct == dev8PID) HWType = 8;  
		if (dev->descriptor.idProduct == dev9PID) HWType = 9;	  
		if (dev->descriptor.idProduct == dev10PID) HWType = 10;  
		if (dev->descriptor.idProduct == dev11PID) HWType = 11;
		if (dev->descriptor.idProduct == dev12PID) HWType = 12;
		if (dev->descriptor.idProduct == dev13PID) HWType = 13;
		if (dev->descriptor.idProduct == dev14PID) HWType = 14;
		if (dev->descriptor.idProduct == dev15PID) HWType = 15;
		if (dev->descriptor.idProduct == dev16PID) HWType = 16;
		if (dev->descriptor.idProduct == dev17PID) HWType = 17;
		if (dev->descriptor.idProduct == dev18PID) HWType = 18;
		if (dev->descriptor.idProduct == dev19PID) HWType = 19;
	}
	  
    if (HWType) { /* we like this device and we'll keep it */
	if (DEBUG_OUT > 1) printf("Opening device %04x:%04x serial %04x type %d\r\n",
			      dev->descriptor.idVendor,
			      dev->descriptor.idProduct,
			      dev->descriptor.iSerialNumber, HWType);
	devhandle = usb_open(dev);
	if (DEBUG_OUT > 1)  printf("LMS device found @ address [%s]\r\n", dev->filename);
	usb_status = usb_get_string_simple(devhandle, dev->descriptor.iSerialNumber, rcvbuff, sizeof(rcvbuff));
	if (DEBUG_OUT > 1) printf("string %d = [%s]\r\n", dev->descriptor.iSerialNumber, rcvbuff);
	if (usb_status < 0) strcpy(HWSerial, ""); else strcpy(HWSerial, rcvbuff+3);
	usb_close(devhandle);
	switch(HWType) {
	case 1: // LMS-103
	  strcpy(HWName, sVNX1);
	  HWMinFrequency = 500000000;			// 5 GHz in 10Hz units
	  HWMaxFrequency = 1000000000;			// 10 GHz max
	  HWMaxPower = HW_MAXP_10;			// +10db output power
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-103 has a 50db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 2:  // LMS-123
	  strcpy(HWName, sVNX2);
	  HWMinFrequency = 800000000; 			// 8 GHz min
	  HWMaxFrequency = 1200000000;			// 12 GHz max
	  HWMaxPower = HW_MAXP_10;			// +10db output power
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-123 has a 50db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 3: // LMS-203
	  strcpy(HWName, sVNX3);
	  HWMinFrequency = 1000000000;			// 10 GHz min
	  HWMaxFrequency = 2000000000;			// 20 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_40;		// the LMS-203 has a 40db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 4: // LMS-802
	  strcpy(HWName, sVNX4);
	  HWMinFrequency = 400000000; 			// 4 GHz min
	  HWMaxFrequency = 800000000; 			// 8 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-802 has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 5: // LMS-163
	  strcpy(HWName, sVNX5);
	  HWMinFrequency = 800000000;			// 8 GHz min
	  HWMaxFrequency = 1600000000;			// 16 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_40;		// the LMS-163 has a 40db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 6: // LMS-322D
	  strcpy(HWName, sVNX6);
	  HWMinFrequency = 60000000;			//  .6 GHz min
	  HWMaxFrequency = 230000000;			// 2.3 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-322D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;  
	case 7: // LMS-232D
	  strcpy(HWName, sVNX7);
	  HWMinFrequency = 50000000;			// .5 GHz min
	  HWMaxFrequency = 230000000;			// 2.3 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-232D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;	  
	case 8: // LMS-602D
	  strcpy(HWName, sVNX8);
	  HWMinFrequency = 150000000;			// 1.5 GHz min
	  HWMaxFrequency = 600000000;			// 6 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-602D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;		  
	case 9: // LMS-402D
	  strcpy(HWName, sVNX9);
	  HWMinFrequency = 100000000;			// 1 GHz min
	  HWMaxFrequency = 400000000;			// 4 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-402D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;		  
	case 10: // LMS-451D
	  strcpy(HWName, sVNX10);
	  HWMinFrequency = 7000000;			// 70  MHz min
	  HWMaxFrequency = 45000000;			// 450 MHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-451D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;	  
	case 11: // LMS-271D
	  strcpy(HWName, sVNX11);
	  HWMinFrequency = 100000;			// 1  MHz min
	  HWMaxFrequency = 27000000;			// 270 MHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-271D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 12: // LMS-152D
	  strcpy(HWName, sVNX12);
	  HWMinFrequency = 25000000;			// 250 MHz min
	  HWMaxFrequency = 150000000;			// 1.5 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-152D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 13: // LMS-751D
	  strcpy(HWName, sVNX13);
	  HWMinFrequency = 10000000;			// 100 MHz min
	  HWMaxFrequency = 75000000;			// 750 MHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-751D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 14: // LMS-252D
	  strcpy(HWName, sVNX14);
	  HWMinFrequency = 40000000;			// 400 MHz min
	  HWMaxFrequency = 250000000;			// 2.5 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_50;		// the LMS-252D has a 50 db output power range
	  HWPowerScale = 1;				// the device uses .25db units 
	  break;
	case 15: // LMS-6123LH
	  strcpy(HWName, sVNX15);
	  HWMinFrequency = 600000000;			// 6 GHz min
	  HWMaxFrequency = 1200000000;			// 12 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_80;		// the LMS-6123LH has a 70 db output power range
	  HWPowerScale = 2;				// the device uses .5db units 
	  break;	  
	case 16: // LMS-163LH
	  strcpy(HWName, sVNX16);
	  HWMinFrequency = 800000000;			// 8 GHz min
	  HWMaxFrequency = 1600000000;			// 16 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_80;		// the LMS-163LH has a 70 db output power range
	  HWPowerScale = 2;				// the device uses .5db units 
	  break;	  
	case 17: // LMS-802LH
	  strcpy(HWName, sVNX17);
	  HWMinFrequency = 200000000;			// 2 GHz min
	  HWMaxFrequency = 800000000;			// 8 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_80;		// the LMS-802LH has a 70 db output power range
	  HWPowerScale = 2;				// the device uses .5db units 
	  break;	  
	case 18: // LMS-802DX
	  strcpy(HWName, sVNX18);
	  HWMinFrequency = 200000000;			// 2 GHz min
	  HWMaxFrequency = 800000000;			// 8 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_80;		// the LMS-802DX has a 70 db output power range
	  HWPowerScale = 2;				// the device uses .5db units 
	  break;	  
	case 19: // LMS-183DX
	  strcpy(HWName, sVNX16);
	  HWMinFrequency = 600000000;			// 6 GHz min
	  HWMaxFrequency = 1800000000;			// 18 GHz max
	  HWMaxPower = HW_MAXP_10;
	  HWMinPower = HW_MAXP_10 - HW_MINP_80;		// the LMS-183DX has a 70 db output power range
	  HWPowerScale = 2;				// the device uses .5db units 
	  break;	  
	  
	} /* HWType switch */
	
	/* find an open slot to save the data */
	// lets see if we have this unit in our table of devices already
	bFound = FALSE;
	
	for (i = 1; i<MAXDEVICES; i++){
	  if (lms[i].SerialNumber == atoi(HWSerial)) {
	    // we already have the device in our table
	    bFound = TRUE;
	    lms[i].DevStatus = lms[i].DevStatus | DEV_CONNECTED; // its here, mark it as connected
	    // at this point the device is present, but not in use, no sense looking more
	    break;
	  }
	}	// end of for loop
	
	// if the device isn't in the table we need to add it
	if (!bFound) {
	  hTemp = 0;
	  for (i = 1; i<MAXDEVICES; i++) {
	    if (lms[i].SerialNumber == 0) {
	      hTemp = i;
	      break;
	    }
	  } // end of for loop search for an empty slot in our array of devices
	  
	  /* save all of the data we've already acquired */
	  if (hTemp) {
		lms[hTemp].LMSType = HWType;				// save the device type to simplify any device specific IO
	    lms[hTemp].SerialNumber = atoi(HWSerial);		        // save the device's serial number
	    lms[hTemp].DevStatus = lms[hTemp].DevStatus | DEV_CONNECTED;// mark it as present
	    strcpy (lms[hTemp].ModelName, HWName);     		    	// save the device's model name
	    //	    lms[hTemp].usb_device_handle = NULL;     		// make sure we're in the unopened state!
	    lms[hTemp].MinFrequency = HWMinFrequency;
	    lms[hTemp].MaxFrequency = HWMaxFrequency;			// default values for frequency range
	    lms[hTemp].MaxPower = HWMaxPower;				// default value for maximum power
	    lms[hTemp].MinPower = HWMinPower;				// and minimum power (adjusted after we read max pwr from device)
	    lms[hTemp].PowerScale = HWPowerScale;			// we need to track the units for each device
		
	    /* The device has been closed so let's make sure we can find it again */
	    lms[hTemp].idVendor = dev->descriptor.idVendor;
	    lms[hTemp].idProduct = dev->descriptor.idProduct;
	    strcpy(lms[hTemp].Serialstr, rcvbuff);
	    if (DEBUG_OUT > 1) {
	      printf("Stored as new device #%d\r\n", hTemp);
	      printf("Serial number=%d\r\n", lms[hTemp].SerialNumber);
	      printf("Devstatus=%08x\r\n", lms[hTemp].DevStatus);
	      printf("Model name=%s\r\n", lms[hTemp].ModelName);
	      printf("MinFrequency=%d\r\n", lms[hTemp].MinFrequency);
	      printf("MaxFrequency=%d\r\n", lms[hTemp].MaxFrequency);
	      printf("MaxPower=%d\r\n", lms[hTemp].MaxPower);
	      printf("MinPower=%d\r\n", lms[hTemp].MinPower);
	      printf("PowerScale=%d\r\n", lms[hTemp].PowerScale);
	      printf("Vendor ID=%04x\r\n", lms[hTemp].idVendor);
	      printf("Product ID=%04x\r\n", lms[hTemp].idProduct);
	      printf("Serial number=%s\r\n", lms[hTemp].Serialstr);
	    }
	  } else {
	    // our table of devices is full, not much we can do
	  }
	} /* if !bfound  */
	/* get any other data we might need */
      } /* if HWType */
    } /* for dev */
  } /* for bus */

  /* clean up the structure and mark unused slots */
  for (i = 1; i<MAXDEVICES; i++){
    if ((lms[i].SerialNumber != 0) && !(lms[i].DevStatus & DEV_CONNECTED))
      lms[i].SerialNumber = 0;	// mark this slot as unused 	

    if (lms[i].SerialNumber == 0)
      lms[i].DevStatus = 0;	// clear the status for robustness!
  }	// end of zombie removal for loop
}

/* ----------------------------------------------------------------- */

void fnLMS_Init(void) {
  /* clear out the storage structure. Must be called once before anything else */
  int i;
  int status;

  for (i = 0; i<MAXDEVICES; i++){
    lms[i].DevStatus = 0; 	// init to no devices connected
    lms[i].SerialNumber = 0;	// clear the serial number
    lms[i].ModelName[0] = 0;	// put a null string in each model name field
  }

  usb_init();
  if (DEBUG_OUT > 0)  printf("library version %s\r\n", fnLMS_LibVersion());
}

void fnLMS_SetTestMode(bool testmode) {
  TestMode = testmode;
}

int fnLMS_GetNumDevices() {
  int retval = 0;
  int NumDevices = 0;
  int i;
  
  // See how many devices we can find, or have found before
  if (TestMode){
    
    // construct a fake device
    lms[1].SerialNumber = 100103;
    lms[1].DevStatus = lms[1].DevStatus | DEV_CONNECTED  | PLL_LOCKED | FAST_PULSE_OPTION;
    lms[1].MinFrequency =  500000000;	// 5Ghz in 10 hz resolution
    lms[1].MaxFrequency = 1000000000;	// 10Ghz in 10 hz resolution
    lms[1].MaxPower = 40;		// 10db in our .25db step system
    lms[1].MinPower = 40 - 200;		// the LMS-103 has a 50 db attenuation range
    lms[1].PowerScale = 1;		// the LMS-103 uses .25db units
    strcpy (lms[1].ModelName, "LMS-103");
    
    // construct a second fake device
    lms[2].SerialNumber = 200123;
    lms[2].DevStatus = lms[2].DevStatus | DEV_CONNECTED  | PLL_LOCKED | FAST_PULSE_OPTION;
    lms[2].MinFrequency = 800000000;	// 8Ghz in 100Khz resolution
    lms[2].MaxFrequency = 1200000000;	// 12Ghz in 100Khz resolution
    lms[2].MaxPower = 40;		// 10db in our .25db step system
    lms[2].MinPower = 40 - 200;		// the '123 has a 50 db attenuation range
    lms[1].PowerScale = 1;
    strcpy (lms[2].ModelName, "LMS-123");

    retval = 2;
    
  } else {
    // go look for some real hardware
    FindVNXDevices();

    // Total up the number of devices we have
    for (i = 0; i < MAXDEVICES; i++){
      if (lms[i].DevStatus & DEV_CONNECTED) NumDevices++; 
    }
    retval = NumDevices;

  }
  return retval;
}

int fnLMS_GetDevInfo(DEVID *ActiveDevices) {
  int i;
  int NumDevices = 0;
  
  if ( ActiveDevices == NULL) return 0;	// bad array pointer, no place to put the DEVIDs
  
  for (i = 1; i < MAXDEVICES; i++){ 	// NB -- we never put an active device in lms[0] - so DEVIDs start at 1

    if (lms[i].DevStatus & DEV_CONNECTED) {
      ActiveDevices[NumDevices] = i;
      NumDevices++;
    }
  }
  
  return NumDevices;
}

int fnLMS_GetModelName(DEVID deviceID, char *ModelName) {
  int NumChars = 0;

  if (deviceID >= MAXDEVICES){
    return 0;
  }

  NumChars = strlen(lms[deviceID].ModelName);
  // If NULL result pointer, just return the number of chars in the name
  if ( ModelName == NULL) return NumChars;
  strcpy(ModelName, lms[deviceID].ModelName);

  return NumChars;
}

int fnLMS_InitDevice(DEVID deviceID) {

  if ((deviceID >= MAXDEVICES) || (deviceID == 0)){
    return INVALID_DEVID;
  }
  
  if (TestMode)
    lms[deviceID].DevStatus = lms[deviceID].DevStatus | DEV_OPENED;
  else {
    // Go ahead and open a handle to the hardware
    if (VNXOpenDevice(deviceID))//VNXOpenDevice returns 0 if the open succeeded
      return DEVICE_NOT_READY;
    if (DEBUG_OUT > 0) printf("Time to start getting device parameters...\r\n");

      // Get the rest of the device parameters from the device
    
    if (!GetFrequency(deviceID))// read the frequency from the device (in 10Hz units)
      return BAD_HID_IO;
  
    if (DEBUG_OUT > 0) printf("Got Frequency...\r\n");

    if (!GetFStart(deviceID))
      return BAD_HID_IO;
    if (DEBUG_OUT > 0) printf("Got FStart...\r\n");

    if (!GetFEnd(deviceID))	// read the sweep end frequency
      return BAD_HID_IO;
      if (DEBUG_OUT > 0) printf("Got FEnd...\r\n");

    if (!GetSweep(deviceID))	// read the sweep time
      return BAD_HID_IO;
      if (DEBUG_OUT > 0) printf("Got Sweep...");

    if (!GetMaxPower(deviceID))
      return BAD_HID_IO;
    if (DEBUG_OUT > 0) printf("Got Max Power...\r\n");

    if (!GetPower(deviceID))
      return BAD_HID_IO;
      if (DEBUG_OUT > 0) printf("Got Power...\r\n");

    if (!GetRF_On(deviceID))
      return BAD_HID_IO;
    if (DEBUG_OUT > 0) printf("Got RF On...\r\n");

    if (!GetUseIntOsc(deviceID))
      return BAD_HID_IO;

    if (!GetUseExtSwpTrigger(deviceID))
      return BAD_HID_IO;
    if (DEBUG_OUT > 0) printf("Got External Sweep Trigger Status...\r\n");
    
    if (!GetFMinimum(deviceID))
      return BAD_HID_IO;
    if (DEBUG_OUT > 0) printf("Got FMinimum...\r\n");

    if (!GetPulseOffTime(deviceID))	// read the parameters for the internal pulse modulation feature
      return BAD_HID_IO;
      if (DEBUG_OUT > 0) printf("Got Pulse Off Time...\r\n");

    if (!GetPulseOnTime(deviceID)) 	// ditto
      return BAD_HID_IO;
    if (DEBUG_OUT > 0) printf("Got Pulse On Time...\r\n");

    if (!GetPulseMode(deviceID)) 	// ditto, put the pulse mode control bits into our Modebits dword, and update device status
      return BAD_HID_IO;		// to show whether or not the Lab Brick has the fast pulse mode option

	// --- calculate the pulse on and off times in seconds from what we read ---

		if ((lms[deviceID].PulseOnTime & 0xF0000000) == 0x10000000)
		{
			lms[deviceID].PulseOnTimeSeconds = ((float) ((lms[deviceID].PulseOnTime & 0x0FFFFFFF)) * 2.083333e-8);
			lms[deviceID].PulseOffTimeSeconds = ((float) ((lms[deviceID].PulseOffTime & 0x0FFFFFFF)) * 2.083333e-8);

		}
		else
		{
			lms[deviceID].PulseOnTimeSeconds = ((float) ((lms[deviceID].PulseOnTime & 0x0FFFFFFF)) * 1.00000e-6);
			lms[deviceID].PulseOffTimeSeconds = ((float) ((lms[deviceID].PulseOffTime & 0x0FFFFFFF)) * 1.00000e-6);
		}
		
	if (DEBUG_OUT > 0) printf("Device Open Successful!\r\n");
	
  } // end of real device open process case

  // if we got here everything worked OK
  return STATUS_OK;
}

int fnLMS_CloseDevice(DEVID deviceID) {
  
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (TestMode)
    lms[deviceID].DevStatus = lms[deviceID].DevStatus & ~DEV_OPENED;
  else {

    // Go ahead and close this hardware - the first step is to stop its read thread
    lms[deviceID].thread_command = THREAD_EXIT;
    
    // The thread handler will close the device. We'll wait up to 1 second then give up.
    int retries;
    retries = 10;
    while (retries && (lms[deviceID].thread_command != THREAD_DEAD)) {
      usleep(100000); /* wait 100 ms */
      retries--;
    }
    if (DEBUG_OUT > 0) printf("After telling the thread to close, we have thread_command=%d retries=%d\r\n", lms[deviceID].thread_command, retries);
    //    threads[deviceID] = NULL;
    lms[deviceID].thread_command = THREAD_IDLE;

    // Mark it closed in our list of devices
    lms[deviceID].DevStatus = lms[deviceID].DevStatus & ~DEV_OPENED;
  }

  return STATUS_OK;

}

int fnLMS_GetSerialNumber(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return 0;
  
  return lms[deviceID].SerialNumber;
}

LVSTATUS fnLMS_SetFrequency(DEVID deviceID, int frequency) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  LockDev(deviceID, TRUE); // prevent updates to the frequency variable reported by the device!
  
  int old_frequency = lms[deviceID].Frequency;
  
  if ((frequency >= lms[deviceID].MinFrequency) && (frequency <= lms[deviceID].MaxFrequency)) {
    lms[deviceID].Frequency = frequency;
    if (TestMode)
      return STATUS_OK;// in test mode we update our internal variables, but don't talk to the real HW
  } else {
    return BAD_PARAMETER;
  }
  // the frequency value is OK, lets send it to the hardware
  unsigned char *ptr = (unsigned char *) &lms[deviceID].Frequency;

  if (DEBUG_OUT > 0) printf("deviceID = %d ptr = %x Frequency = %x\r\n", deviceID , ptr, *ptr);

  lms[deviceID].pending = 1;
  usleep(10000); // wait 10 ms for any reads to stop
  if (!SendReport(deviceID, VNX_DSS_FREQUENCY | VNX_SET, ptr, 4)){

    lms[deviceID].Frequency = old_frequency;
    LockDev(deviceID, FALSE);					// We're done using the frequency variables, unlock them..
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  LockDev(deviceID, FALSE);// We're done, let the status reports update frequency (in case of a sweep)
  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_SetStartFrequency(DEVID deviceID, int startfrequency) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  int old_startfrequency = lms[deviceID].StartFrequency;

  if ((startfrequency >= lms[deviceID].MinFrequency) && (startfrequency <= lms[deviceID].MaxFrequency)) {
    lms[deviceID].StartFrequency = startfrequency;
    if (TestMode)
      return STATUS_OK;// in test mode we update our internal variables, but don't talk to the real HW
  } else
    return BAD_PARAMETER;

  // the sweep start frequency value is OK, lets send it to the hardware
  unsigned char *ptr = (unsigned char *) &lms[deviceID].StartFrequency;

  lms[deviceID].pending = 1;
  usleep(10000); /* wait 10 ms for any reads to stop */
  if (DEBUG_OUT > 0) printf("deviceID = %d ptr = %x StartFrequency = %x\r\n", deviceID, ptr, *ptr);
  if (!SendReport(deviceID, VNX_DSS_FSTART | VNX_SET, ptr, 4)){
    lms[deviceID].StartFrequency = old_startfrequency;
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_SetEndFrequency(DEVID deviceID, int endfrequency) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  int old_endfrequency = lms[deviceID].EndFrequency;

  if ((endfrequency >= lms[deviceID].MinFrequency) && (endfrequency <= lms[deviceID].MaxFrequency)) {
    lms[deviceID].EndFrequency = endfrequency;
    if (TestMode)
      return STATUS_OK;// in test mode we update our internal variables, but don't talk to the real HW
  } else
    return BAD_PARAMETER;

  // the sweep end frequency value is OK, lets send it to the hardware
  unsigned char *ptr = (unsigned char *) &lms[deviceID].EndFrequency;

  if (DEBUG_OUT > 0) printf("deviceID = %d ptr = %x SweepEndFrequency = %x\r\n", deviceID, ptr, *ptr);
  
  lms[deviceID].pending = 1;
  usleep(10000); // wait 10 ms for any reads to stop
  if (!SendReport(deviceID, VNX_DSS_FSTOP | VNX_SET, ptr, 4)){
    lms[deviceID].EndFrequency = old_endfrequency;
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_SetSweepTime(DEVID deviceID, int sweeptime) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  int old_sweeptime = lms[deviceID].SweepTime;

  if (sweeptime >= VNX_MIN_SWEEPTIME) {
    lms[deviceID].SweepTime = sweeptime;
    if (TestMode)
      return STATUS_OK;// in test mode we update our internal variables, but don't talk to the real HW
  } else
      return BAD_PARAMETER;
 
  // the sweep time value is OK, lets send it to the hardware
  unsigned char *ptr = (unsigned char *) &lms[deviceID].SweepTime;

  if (DEBUG_OUT > 0) printf("deviceID = %d ptr = %x SweepTime = %x\r\n", deviceID, ptr, *ptr);
  
  lms[deviceID].pending = 1;
  usleep(10000); // wait 10 ms for any reads to stop
  if (!SendReport(deviceID, VNX_DSS_SWPTIME | VNX_SET, ptr, 4)){
    lms[deviceID].SweepTime = old_sweeptime;
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_SetPowerLevel(DEVID deviceID, int powerlevel) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  int old_powerlevel = lms[deviceID].Power;
  int ScaledAttenuation;

  // We use an absolute power level for the API, so the attenuation is computed from the value set by the user
  if (DEBUG_OUT > 1) printf("MaxPower = %d MinPower = %d \r\n", lms[deviceID].MaxPower, lms[deviceID].MinPower);  


  // First off, check if the desired power setting is in range
  if ((powerlevel > lms[deviceID].MaxPower) || (powerlevel < lms[deviceID].MinPower))
    return BAD_PARAMETER;	// power setting is out of range, bail out

  // Save the new power level setting
  lms[deviceID].Power = powerlevel;
  if (TestMode)
    return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW

  // We set the attenuation relative to MaxPower, so we have to subtract the power level value to get the attenuation
  // for power level settings below 0 db we end up adding the value since it is negative.
  
  powerlevel = (lms[deviceID].MaxPower - powerlevel);
  
  if ((powerlevel <= 0) || (powerlevel >= 4 * (lms[deviceID].LMSType == 15 ? MAX_ATTEN_LH : MAX_ATTEN)))
    return BAD_PARAMETER;	// we should not see this case unless min or max power are mangled, but we're defending against
				// any possible bad internal attenuation command values

  // the power level value is OK, lets scale it and then send it to the hardware
  if (lms[deviceID].PowerScale) ScaledAttenuation = powerlevel / lms[deviceID].PowerScale;	// defend against (unlikely) bad LMSPARAMS data
  else ScaledAttenuation = powerlevel;

  unsigned char *ptr = (unsigned char *) &ScaledAttenuation;

  if (DEBUG_OUT > 0) printf("deviceID = %d ptr = %x PowerLevel = %x\r\n", deviceID, ptr, *ptr);
  lms[deviceID].pending = 1;
  usleep(10000); /* wait 10 ms for any reads to stop */
  if (!SendReport(deviceID, VNX_PWR | VNX_SET, ptr, 4)){
    lms[deviceID].Power = old_powerlevel;
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_SetRFOn(DEVID deviceID, bool on) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
      return DEVICE_NOT_READY;

  char VNX_command[4] = {0, 0, 0, 0};

  if (on) {
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_RFON;
    lms[deviceID].Modebits = lms[deviceID].Modebits | MODE_RFON;
    VNX_command[0] = 1;
  } else { // off
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_RFON;
    VNX_command[0] = 0;
  }

  if (TestMode)
    return STATUS_OK; // in test mode we update our internal variables, but don't talk to the real HW

  lms[deviceID].pending = 1;
  usleep(10000); /* wait 10 ms for any reads to stop */
  if (!SendReport(deviceID, VNX_RFMUTE | VNX_SET, VNX_command, 1)) {
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }

  lms[deviceID].pending = 0;
  return STATUS_OK;
}

// *** revised in V1.01 to match the Windows API, pulseontime is now a floating point value
// This code converts from the floating point on time value (in seconds) to the ranged integer units
// it does not check for required relationship with off time.
LVSTATUS fnLMS_SetPulseOnTime(DEVID deviceID, float pulseontime) {
  int temp_pulseontime;
  
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  float old_pulseontime = lms[deviceID].PulseOnTimeSeconds;

  if (pulseontime >= 0.10e-6 && pulseontime < 1000){		// pulse times can range from 100ns to 1000 seconds
	lms[deviceID].PulseOnTimeSeconds = pulseontime;
    if (TestMode)
      return STATUS_OK;	// in test mode we update our internal variables, but don't talk to the real HW
  } else
    return BAD_PARAMETER;

if (pulseontime <= .001)		// use the 48MHz clock
	{
		// generate the integer on time
		temp_pulseontime = (int) (48 * (pulseontime * 1.0e+6));
		temp_pulseontime |= PM48Mhz;
	}
	else
	{
		temp_pulseontime = (int) (pulseontime * 1.0e+6);
	}

  // the pulse on time value is OK (as far as we can tell...), lets send it to the hardware
  unsigned char *ptr = (unsigned char *) &temp_pulseontime;

  if (DEBUG_OUT > 0) printf("deviceID = %d ptr = %x Pulse On Time = %x\r\n", deviceID, ptr, *ptr);
  lms[deviceID].pending = 1;
  usleep(10000); // wait 10 ms for any reads to stop

  if (!SendReport(deviceID, VNX_ONTIME | VNX_SET, ptr, 4)){
    lms[deviceID].PulseOnTime = old_pulseontime;
	lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  
  lms[deviceID].PulseOnTime = temp_pulseontime;		// we keep a copy of the encoded on time value in case we need it later
  lms[deviceID].pending = 0;
  return STATUS_OK;
}
// *** revised in V1.01 to match the Windows API, pulseontime is now a floating point value
// This code converts from the floating point on time value (in seconds) to the ranged integer units
// it does not check for required relationship with on time - the calling application must ensure that the 
// on and off times will have the same encoding.
LVSTATUS fnLMS_SetPulseOffTime(DEVID deviceID, float pulseofftime) {
   int temp_pulseofftime;

  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  float old_pulseofftime = lms[deviceID].PulseOffTimeSeconds;
  
// ------ make sure parameter is in range ----------
  if (pulseofftime >= 150.0e-9 && pulseofftime < 1000){
	lms[deviceID].PulseOffTimeSeconds = pulseofftime;
	if (TestMode) return STATUS_OK;	// in test mode we update our internal variables, but don't talk to the real HW
  }
  else
  {
	return BAD_PARAMETER;			// we end up here if the off time is less than 150ns or greater than 1000 sec
  }

  // find the encoding for the new pulse off time
  if (pulseofftime <= .001)		// use the 48MHz clock
	{
		// generate the integer off time, it does not have the range flags!!
		temp_pulseofftime = (int) (48 * (pulseofftime * 1.0e+6));
	}
	else
	{ 
		temp_pulseofftime = (int) (pulseofftime * 1.0e+6);
	}
	
  // the pulse off time value is OK, lets send it to the hardware
  unsigned char *ptr = (unsigned char *) &temp_pulseofftime;

  if (DEBUG_OUT > 0) printf("deviceID = %d ptr = %x Pulse Off Time = %x\r\n", deviceID, ptr, *ptr);
  lms[deviceID].pending = 1;
  usleep(10000); // wait 10 ms for any reads to stop
  if (!SendReport(deviceID, VNX_OFFTIME | VNX_SET, ptr, 4)) {
    lms[deviceID].PulseOffTimeSeconds = old_pulseofftime;
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }

  lms[deviceID].PulseOffTime = temp_pulseofftime;	// we keep a copy of the encoded off time value in case we need it later
  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_EnableInternalPulseMod(DEVID deviceID, bool on) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  char VNX_command[4] = {0, 0, 0, 0};

  if (on)
    lms[deviceID].Modebits = lms[deviceID].Modebits | MODE_PWMON;
  else
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_PWMON;

  VNX_command[0] = (lms[deviceID].Modebits & PWM_MASK) >> 8;

  if (TestMode)
    return STATUS_OK;// in test mode we update our internal variables, but don't talk to the real HW

  lms[deviceID].pending = 1;
  usleep(10000); /* wait 10 ms for any reads to stop */
  if (!SendReport(deviceID, VNX_PULSE_MODE | VNX_SET, VNX_command, 1)) {
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }

  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_SetUseExternalPulseMod(DEVID deviceID, bool external) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  char VNX_command[4] = {0, 0, 0, 0};

  if (external)
    lms[deviceID].Modebits = lms[deviceID].Modebits | MODE_EXTPWM;
  else
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_EXTPWM;

  VNX_command[0] = (lms[deviceID].Modebits & PWM_MASK) >> 8;

  if (TestMode)
    return STATUS_OK; // in test mode we update our internal variables, but don't talk to the real HW

  lms[deviceID].pending = 1;
  usleep(10000); /* wait 10 ms for any reads to stop */
  if (!SendReport(deviceID, VNX_PULSE_MODE | VNX_SET, VNX_command, 1)) {
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }

  lms[deviceID].pending = 0;
  return STATUS_OK;
}

// *** Implemented in V1.01 to match the Windows API ***
LVSTATUS fnLMS_SetFastPulsedOutput(DEVID deviceID,  float pulseontime, float pulsereptime, bool on) {
  int temp_pulseontime;
  int temp_pulseofftime;
  char VNX_command[4] = {0, 0, 0, 0};

  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  float old_pulseontime = lms[deviceID].PulseOnTimeSeconds;
  float old_pulseofftime = lms[deviceID].PulseOffTimeSeconds;
  
  if (pulsereptime <= pulseontime) 						// the on time has to be less than the repetition time
    return BAD_PARAMETER;

// ------ first we have to convert from the floating point times to our integers ---------------------
// we have to pick the range of units depending on whether or not we can use the 48MHz, 1 MHz or 1ms clocks
  if (DEBUG_OUT > 0) printf ("pulseontime = %f, pulsereptime = %f\r\n", pulseontime, pulsereptime);

  if (fabs(pulsereptime - .001) <= .00001)				// use the 48MHz clock
  {
	if ((pulseontime + 5.0e-9 > pulsereptime))			// the on time plus one clock has to be less than the repetition time
	{
	  return BAD_PARAMETER;
	}

	// generate the integer on time
	if (DEBUG_OUT > 0) printf ("using 48MHz clock\r\n");
	temp_pulseontime = (int) (48 * (pulseontime * 1.0e+6));
	temp_pulseofftime = (int) ((48 * (pulsereptime * 1.0e+6)) - temp_pulseontime);

	if (DEBUG_OUT > 0) printf (" 48MHZ temp_pulseontime = %d, temp_pulseofftime = %d\r\n", temp_pulseontime, temp_pulseofftime);
	if (temp_pulseofftime <= 0) return BAD_PARAMETER;
		
	temp_pulseontime |= PM48Mhz;							// add our units encoding marker
	
	}
	else if (pulsereptime > .001 && pulsereptime <= .050)	// use the 1MHz clock
	{
		if ((pulseontime + 1.0e-6 > pulsereptime))			// the on time plus one clock has to be less than the repetition time
		{
		  return BAD_PARAMETER;
		}

		if (DEBUG_OUT > 0)printf ("using 1MHz clock\r\n");

		temp_pulseontime = (int) (pulseontime * 1.0e+6);
		temp_pulseofftime = (int) (pulsereptime * 1.0e+6) - temp_pulseontime;

		if (temp_pulseofftime <= 0) return BAD_PARAMETER;
		if (DEBUG_OUT > 0) printf (" 1MHZ temp_pulseontime = %d, temp_pulseofftime = %d\r\n", temp_pulseontime, temp_pulseofftime);
	}
	else		// for rep time > 50 ms we use the software timer (its really the same as the above case)
	{

		if ((pulseontime + .001 > pulsereptime))		// the on time plus one clock has to be less than the repetition time
		{
		  return BAD_PARAMETER;
		}

		if (pulsereptime > 1000) return BAD_PARAMETER;	// maximum time allowed by API is 1000 seconds
		if (DEBUG_OUT > 0) printf ("using 1 ms. timer\r\n");

		// ---- we represent the time in 1 microsecond units --------
		temp_pulseontime = (int) (pulseontime * 1.0e+6);
		temp_pulseofftime = (int) (pulsereptime * 1.0e+6) - temp_pulseontime;
		
		if (DEBUG_OUT > 0) printf (" SW temp_pulseontime = %d, temp_pulseofftime = %d\r\n", temp_pulseontime, temp_pulseofftime);
		if (temp_pulseofftime <= 0) return BAD_PARAMETER;
	}

	// At this point we can update our local copies of the on and off time in seconds
	// We'll restore the old values if somehow the hardware I/O fails
		lms[deviceID].PulseOnTimeSeconds = pulseontime; 
		lms[deviceID].PulseOffTimeSeconds = pulsereptime - pulseontime;

	if (DEBUG_OUT > 0) printf (" Setting PulseOnTimeSeconds = %f, PulseOffTimeSeconds = %f\r\n", lms[deviceID].PulseOnTimeSeconds, lms[deviceID].PulseOffTimeSeconds); 

	// Now send the parameters to the device if we aren't in test mode
	if (TestMode)
	{
		if (on){				// keep the mode bits in sync
			lms[deviceID].Modebits = lms[deviceID].Modebits | MODE_PWMON;
		}
		else
		{
			lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_PWMON;
		}
		return STATUS_OK;		// in test mode we update our internal variables, but don't talk to the real HW
	}

	// First we disable any active fast pulse mode operation, including external mode ...
	VNX_command[0] = 0;			// pulse mode off, internal pulse modulation selected
	lms[deviceID].pending = 1;
	usleep(10000); 				// wait 10 ms for any reads to stop
	if (!SendReport(deviceID, VNX_PULSE_MODE | VNX_SET, VNX_command, 1))
	{
		// -- our IO failed, so leave the old settings and bail out --
		//	  we can't do much about restoring the pulse mode state...
		lms[deviceID].PulseOnTimeSeconds = old_pulseontime; 
		lms[deviceID].PulseOffTimeSeconds = old_pulseofftime;
		lms[deviceID].pending = 0;
		return BAD_HID_IO;
	}

	// Then we send the on time, followed by the off time
	unsigned char *ptr = (unsigned char *) &temp_pulseontime;

	if (DEBUG_OUT > 0) printf("deviceID = 0x%x ptr = 0x%x Pulse On Time LSByte = 0x%x PulseOnTime = %d\r\n", deviceID, ptr, *ptr, temp_pulseontime);

	usleep(5000); 				// wait 5 ms to avoid same frame command writes
	if (!SendReport(deviceID, VNX_ONTIME | VNX_SET, ptr, 4)){

		// -- our IO failed, so leave the old settings and bail out --
		lms[deviceID].PulseOnTimeSeconds = old_pulseontime; 
		lms[deviceID].PulseOffTimeSeconds = old_pulseofftime;
		lms[deviceID].pending = 0;
		return BAD_HID_IO;
	}

	ptr = (unsigned char *) &temp_pulseofftime;
	if (DEBUG_OUT > 0) printf("deviceID = 0x%x ptr = 0x%x Pulse Off Time LSByte = 0x%x PulseOffTime = %d\r\n", deviceID, ptr, *ptr, temp_pulseofftime);

	usleep(5000); 				// wait 5 ms to avoid same frame command writes
	if (!SendReport(deviceID, VNX_OFFTIME | VNX_SET, ptr, 4)){

		// -- we're in a pickle here, we set the new pulse on time, but failed on the new off time setting
		//    so our state variables may not be viable value wise, but since talking to the device is failing
		//    we really can't do much about it!
		lms[deviceID].PulseOffTimeSeconds = old_pulseofftime;
		lms[deviceID].pending = 0;		
		return BAD_HID_IO;
	}

	// -- time to activate the pulse mode operation with the new settings --
	if (on){
		lms[deviceID].Modebits = lms[deviceID].Modebits | MODE_PWMON;
	}
	else
	{
		lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_PWMON;
	}

	VNX_command[0] = (lms[deviceID].Modebits & PWM_MASK) >> 8;

	usleep(5000); 				// wait 5 ms to avoid same frame command writes
	if (!SendReport(deviceID, VNX_PULSE_MODE | VNX_SET, VNX_command, 1)){

		// -- a failure here leaves the settings intact, and in sync, except for the mode bits
		//    probably not worth worrying about trying to restore them since who knows what is
		//    going on below us to cause the failure...
		lms[deviceID].pending = 0;
		return BAD_HID_IO;
	}
	lms[deviceID].pending = 0;			// we're done sending commands, re-enable the reads
	
	if (DEBUG_OUT > 0) printf("Pulse Mode Set to: 0x%x\n", VNX_command[0]); 

	return STATUS_OK;
}

LVSTATUS fnLMS_SetUseInternalRef(DEVID deviceID, bool internal) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  char VNX_command[4] = {0, 0, 0, 0};

  if (internal) {
    lms[deviceID].Modebits = lms[deviceID].Modebits | MODE_INTREF;
    VNX_command[0] = 1;
  } else {
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_INTREF;
    VNX_command[0] = 0;
  }

  if (TestMode)
    return STATUS_OK; // in test mode we update our internal variables, but don't talk to the real HW

  lms[deviceID].pending = 1;
  usleep(10000); // wait 10 ms for any reads to stop
  if (!SendReport(deviceID, VNX_INTOSC | VNX_SET, VNX_command, 1)) {
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  
  lms[deviceID].pending = 0;  
  return STATUS_OK;
}

LVSTATUS fnLMS_SetUseExtSweepTrigger(DEVID deviceID, bool external) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  char VNX_command[4] = {0, 0, 0, 0};

  if (external) {
    lms[deviceID].Modebits = lms[deviceID].Modebits | MODE_EXTSWEEP;
    VNX_command[0] = 1;
  } else {
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~MODE_EXTSWEEP;
    VNX_command[0] = 0;
  }

  if (TestMode)
    return STATUS_OK; // in test mode we update our internal variables, but don't talk to the real HW

  lms[deviceID].pending = 1;
  usleep(10000); // wait 10 ms for any reads to stop
  if (!SendReport(deviceID, VNX_EXTSWP | VNX_SET, VNX_command, 1)) {
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  
  lms[deviceID].pending = 0;  
  return STATUS_OK;
}


LVSTATUS fnLMS_SetSweepDirection(DEVID deviceID, bool up) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  if (up)
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~SWP_DIRECTION;	// sweep direction up (bit == 0)
  else
    lms[deviceID].Modebits = lms[deviceID].Modebits | SWP_DIRECTION;	// sweep direction downwards
  
  return STATUS_OK;
}

LVSTATUS fnLMS_SetSweepMode(DEVID deviceID, bool mode) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (mode) {
    lms[deviceID].Modebits = lms[deviceID].Modebits | SWP_CONTINUOUS;// Repeated sweep
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~SWP_ONCE;
  } else {
    lms[deviceID].Modebits = lms[deviceID].Modebits | SWP_ONCE;// one time sweep
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~SWP_CONTINUOUS;
  }

  return STATUS_OK;
}

LVSTATUS fnLMS_SetSweepType(DEVID deviceID, bool swptype) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (swptype)
    lms[deviceID].Modebits = lms[deviceID].Modebits | SWP_BIDIR;// sweep bidirectionally 
  else
    lms[deviceID].Modebits = lms[deviceID].Modebits & ~SWP_BIDIR;// sawtooth shaped sweep envelope

  return STATUS_OK;
}

LVSTATUS fnLMS_StartSweep(DEVID deviceID, bool go) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  char VNX_sweep[4] = {0, 0, 0, 0};

  if (go)
    VNX_sweep[0] = (char)lms[deviceID].Modebits & MODE_SWEEP;
  else
    VNX_sweep[0] = 0;
  
  if (TestMode)
    return STATUS_OK;// in test mode we update our internal variables, but don't talk to the real HW

  if (DEBUG_OUT > 0) printf(" sending a sweep command = %x\r\n", VNX_sweep[0] );

  lms[deviceID].pending = 1;
  usleep(10000); /* wait 10 ms for any reads to stop */
  if (!SendReport(deviceID, VNX_SWEEP | VNX_SET, VNX_sweep, 1)) {
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  
  lms[deviceID].pending = 0;
  return STATUS_OK;
}

LVSTATUS fnLMS_SaveSettings(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (TestMode)
    return STATUS_OK;// in test mode we update our internal variables, but don't talk to the real HW

  char VNX_savesettings[3] = {0x42, 0x55, 0x31}; //three byte key to unlock the user protection.

  lms[deviceID].pending = 1;
  usleep(10000); /* wait 10 ms for any reads to stop */
  if (!SendReport(deviceID, VNX_SAVEPAR | VNX_SET, VNX_savesettings, 3)) {
    lms[deviceID].pending = 0;
    return BAD_HID_IO;
  }
  
  lms[deviceID].pending = 0;  
  return STATUS_OK;
}

int fnLMS_GetFrequency(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  return lms[deviceID].Frequency;
}

int fnLMS_GetStartFrequency(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
      return DEVICE_NOT_READY;
  
  return lms[deviceID].StartFrequency;
}

int fnLMS_GetEndFrequency(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  return lms[deviceID].EndFrequency;
}

int fnLMS_GetSweepTime(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  return lms[deviceID].SweepTime;
}

int fnLMS_GetRF_On(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
      return DEVICE_NOT_READY;

  if (lms[deviceID].Modebits & MODE_RFON)
    return 1;
  else
    return 0;
}

int fnLMS_GetUseInternalRef(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (lms[deviceID].Modebits & MODE_INTREF)
    return 1;
  else
    return 0;
}

int fnLMS_GetUseExtSweepTrigger(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  if (lms[deviceID].Modebits & MODE_EXTSWEEP)
    return 1;
  else
    return 0;
}

int fnLMS_GetPowerLevel(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  return lms[deviceID].Power;
}

int fnLMS_GetMaxPwr(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;
  
  return lms[deviceID].MaxPower;
}

int fnLMS_GetMinPwr(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
    return DEVICE_NOT_READY;

  return lms[deviceID].MinPower;}

int fnLMS_GetMaxFreq(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;

  return lms[deviceID].MaxFrequency;
}

int fnLMS_GetMinFreq(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return INVALID_DEVID;
  
  return lms[deviceID].MinFrequency;
}
// *** Changed in V1.01 to match the Windows API ***
float fnLMS_GetPulseOnTime(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return F_INVALID_DEVID;
  
  if (!CheckDeviceOpen(deviceID))
    return F_DEVICE_NOT_READY;

  return lms[deviceID].PulseOnTimeSeconds;
}
// *** Changed in V1.01 to match the Windows API ***
float fnLMS_GetPulseOffTime(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
    return F_INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
      return F_DEVICE_NOT_READY;

  return lms[deviceID].PulseOffTimeSeconds;
}
// *** Implemented in V1.01 to match the Windows API ***
int fnLMS_GetPulseMode(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
	return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
	return DEVICE_NOT_READY;

  if (lms[deviceID].Modebits & MODE_PWMON)
	return 1;
  else
	return 0;
}

// *** Implemented in V1.01 to match the Windows API ***
// Find out if we are using the internal pulse modulation source
int fnLMS_GetUseInternalPulseMod(DEVID deviceID) {
  if (deviceID >= MAXDEVICES)
	return INVALID_DEVID;

  if (!CheckDeviceOpen(deviceID))
	return DEVICE_NOT_READY;

  if (lms[deviceID].Modebits & MODE_EXTPWM)
	return 0;	// Note -- the flag is true for external PWM source, in the DLL API we 
				//         report use of the internal source as true.
  else
	return 1;
}
