#include <xbee_serial.h>
#include <unistd.h>  // read / write
#include <simple_serial.h>
#include <rc/start_stop.h>
#include <stdio.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <thread_defs.h>

// static pthread_t serial_read_thread;
// int * fd;

int serial_start() {
  // unsigned char c = 'D';

  // if (argc != 3) {
  //   printf("Usage: " << endl;
  //   printf(" ./receiveSerial <Serial Port> <Baud Rate> \n\n")

  //   printf(" <Serial Port> = /dev/ttyUSB0, etc..." << endl;
  //   printf(" <Baud Rate> = 9600,115200, etc..." << endl;

  //   return 1;
  // }

  fd = serial_open("/dev/ttyGS0", 115200, 0);  // blocking == 1 now,

  if (fd == -1) {
    fprintf(stderr,"Failed to open Serial Port");
    return -1;
  }
  return 0;
}

static void* serial_read(void* ptr) {
  unsigned char z = 'D';
  while (1) {
    if (read(fd, &z, 1) > 0) {
      printf("%c", z);
    }
  }
  return;
}


int serialer()
{
    if (serial_start() != 0) {
      fprintf(stderr, "ERROR in opening serial\n");
        return -1;
    }
    // start thread
    if (rc_pthread_create(
            &serial_read_thread, &serial_read, NULL, SCHED_FIFO, INPUT_MANAGER_PRI) == -1) {
        fprintf(stderr, "ERROR in input_manager_init, failed to start thread\n");
        return -1;
    }
}