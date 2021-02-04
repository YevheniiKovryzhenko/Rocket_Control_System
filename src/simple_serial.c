#include <xbee_serial.h>
#include <unistd.h>  // read / write
#include <simple_serial.h>
#include <rc/start_stop.h>
#include <stdio.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <thread_defs.h>


void* serial_read(void* ptr) {
  int *fd = (int *)ptr;
  unsigned char z = 'D';

  while (rc_get_state() != EXITING) {
    if (read(*fd, &z, 1) > 0) {
      printf("%c", z);
    }
  }
  
  return NULL;
}


int simple_serial_init(struct simple_serial_t *serial_device) {
    int *fd = serial_open(serial_device->port, serial_device->baud_rate, 0);
    if (*fd == -1) {
      fprintf(stderr, "ERROR in opening serial\n");
        return -1;
    }

    // start thread
    if (rc_pthread_create(
          &(serial_device->serial_read_thread), &serial_read, (void *)fd, SCHED_FIFO, INPUT_MANAGER_PRI) == -1) {
        fprintf(stderr, "ERROR in simple_serial_init, failed to start thread\n");
        return -1;
    }

    return 0;
}

int simple_serial_cleanup(struct simple_serial_t *serial_device) {
  if (rc_pthread_timed_join(serial_device->serial_read_thread, NULL, INPUT_MANAGER_TOUT) == 1)
  {
    fprintf(stderr, "WARNING: in serialer_cleanup, thread join timeout\n");
    return -1;
  }
  return 0;
}