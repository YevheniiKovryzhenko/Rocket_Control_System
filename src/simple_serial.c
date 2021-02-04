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
  fd = serial_open("/dev/ttyACM0", 115200, 0);  // blocking == 1 now,

  if (fd == -1) {
    fprintf(stderr,"Failed to open Serial Port");
    return -1;
  }
  return 0;
}

void* serial_read(void* ptr) {
  unsigned char z = 'D';

  while (rc_get_state() != EXITING) {
    if (read(fd, &z, 1) > 0) {
      printf("%c", z);
    }
  }
  
  return NULL;
}


int simple_serial_init() {
    if (serial_start() == -1) {
      fprintf(stderr, "ERROR in opening serial\n");
        return -1;
    }
    // start thread
    if (rc_pthread_create(
            &serial_read_thread, &serial_read, NULL, SCHED_FIFO, INPUT_MANAGER_PRI) == -1) {
        fprintf(stderr, "ERROR in simple_serial_init, failed to start thread\n");
        return -1;
    }

    return 0;
}

int simple_serial_cleanup() {
  if (rc_pthread_timed_join(serial_read_thread, NULL, INPUT_MANAGER_TOUT) == 1)
  {
    fprintf(stderr, "WARNING: in serialer_cleanup, thread join timeout\n");
    return -1;
  }
  return 0;
}