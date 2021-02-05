#include <xbee_serial.h>
#include <unistd.h>  // read / write
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <thread_defs.h>

struct simple_serial_t {
    pthread_t serial_read_thread;
    char port[20];
    int baud_rate;
};

void* serial_read(void* ptr);

int simple_serial_init(struct simple_serial_t *serial_device);
int simple_serial_cleanup(struct simple_serial_t *serial_device);