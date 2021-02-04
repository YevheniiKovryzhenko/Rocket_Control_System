#include <xbee_serial.h>
#include <unistd.h>  // read / write
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <thread_defs.h>

static pthread_t serial_read_thread;
int fd;
static int thread_ret_val;

int serial_start();

void* serial_read(void* ptr);

int simple_serial_init();
int simple_serial_cleanup();