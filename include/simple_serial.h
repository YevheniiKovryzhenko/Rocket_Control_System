#include <serial.h>
#include <unistd.h>  // read / write
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <thread_defs.h>

extern static pthread_t serial_read_thread;
extern int * fd;

int serial_start(char* serial_port, int baud_rate);

void serial_read();

int serialer();