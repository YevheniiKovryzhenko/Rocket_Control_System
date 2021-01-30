#include <xbee_serial.h>
#include <unistd.h>  // read / write
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <thread_defs.h>

static pthread_t serial_read_thread;
int fd;

int serial_start();

static void* serial_read(void* ptr);

int serialer();