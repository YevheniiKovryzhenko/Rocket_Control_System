// Updated code (Feb 2019)
//
// Using data structure for packets with:
// Two start bytes:  0x81, 0xA1
// [Not included: Message ID (one byte), Message payload size (one byte) since we only have one message type]
// Message data (xbee_packet_t length)
// Fletcher-16 checksum (two bytes) computed starting with Message payload size
//
// Note:  This MBin protocol is commonly used on embedded serial devices subject to errors

#include <stdio.h>
#include <unistd.h> // read / write
#include <stdlib.h>	//one of these two is for memcpy
#include <string.h>
#include <stdint.h>

#include <rc/time.h> // for nanos

#include <xbee_serial.h>

// Below for PRId64
#include <inttypes.h>

#include "fallback_packet.h"
fallback_packet_t serialMsg;

int serial_portID;  // Defined as extern in xbee_packet_t.h

// Information local to this file
void readRingBuffer();
#define RING_BUFSIZE 256*2
#define RING_INC(a)(a<(RING_BUFSIZE-1))?(a+1):0  // Increment ring buffer index macro
int ring_overflow=0, rdIndex=0, wrIndex=0;
unsigned char ringbuffer[RING_BUFSIZE];

int serial_init() {
  int baudRate = 230400;  
  serial_portID = serial_open("/dev/ttyS1",baudRate,0); // Nonblocking = 0, BLOCKING = 1 
  if(serial_portID == -1)    {
    printf("Failed to open Serial Port\n");
    return -1;
  }
  return 0;
}


// Read message received from XBee; use ring buffer to assure no data loss
int serial_getData()
{
  int k;
  unsigned char buffer;
  // Populate ring buffer
  for (k=0;k<RING_BUFSIZE;k++) {
    if (ring_overflow) { // Overflow condition on last read attempt
      if (rdIndex == wrIndex) { // Indices equal:  overflow still in effect
	return -1;  // Do not read data; reading data would cause overflow
	break;
      } else
	ring_overflow = 0;  // Reset overflow flag
    }

    if (read(serial_portID, &buffer, 1) > 0) { // Read from buffer: returns 0 or -1 if no data
      ringbuffer[wrIndex] = buffer;
	  wrIndex = RING_INC(wrIndex);
      if (wrIndex == rdIndex) ring_overflow = 1;
    } else
      break;
  }

  // Read and process data in ring buffer; print data
  readRingBuffer();
  return 0;
}



// readRingBuffer()
#define startByte1 0x81
#define startByte2 0xA1

uint64_t last_time;

void readRingBuffer()
{
  static unsigned char msgState = 0, msglength = 0;
  static unsigned char msgdata[SERIAL_DATA_LENGTH], ck0, ck1;
  //printf("\n Trying to read data \n");

  while(ring_overflow || (rdIndex != wrIndex)) { //Don't get ahead of the receiving data 
    if (ring_overflow) ring_overflow = 0; // Reset buffer overflow flag
    // Case 0:  Current character is first message header byte
    if((ringbuffer[rdIndex] == startByte1) && !msgState) {
	    //printf("\n Received Start byte!\n"); //test if xbee is working and receiving start byte
      msgState = 1; 
      msglength = 0;
    }
     
    // Case 1:  Current character is second message header byte
    else if (msgState == 1) {
      if (ringbuffer[rdIndex] == startByte2) msgState = 2; 
      else msgState = 0;  // Bad message
      ck0 = 0;  // Initialize Fletcher checksum bytes
      ck1 = 0;
      msglength = 0;  // Initialize number of message payload data bytes read thusfar
    }
       
    // Case 2:  Read data bytes into msgdata[] array
    else if (msgState == 2) {
      msgdata[msglength++] = ringbuffer[rdIndex];
      ck0 += ringbuffer[rdIndex];
      ck1 += ck0;

      //printf("\n byte = %X", ringbuffer[rdIndex]);
      if (msglength == SERIAL_DATA_LENGTH) msgState = 3; // Done reading data
    }
    
    // Case 3:  Read & check the first checksum byte
    // Throw away data if full checksum doesn't match
    else if (msgState == 3) {
      if (ck0 != ringbuffer[rdIndex]) msgState = 0;
      else msgState = 4;
      //printf("\n ck0 = %X and byte = %X", ck0, ringbuffer[rdIndex]);
    }

    // Case 4:  Read & check the second checksum byte
    else if (msgState == 4) {
        msgState = 0;  // Done reading message data
        if (ck1 == ringbuffer[rdIndex])
        { // Valid message -- copy and print --- must figure out the checksum later (JK)

            double dt_s = (rc_nanos_since_boot() - last_time) / (1e9); //calculate time since last successful reading
            if (1 / dt_s < 20) printf("\nWARNING, Low update frequency of Xbee %f (Hz)\n", 1 / dt_s); //check the update frequency
            memcpy(&serialMsg, msgdata, SERIAL_DATA_LENGTH);

            last_time = rc_nanos_since_boot();

        }
    }
    rdIndex = RING_INC(rdIndex);
  }  
  return;
}





