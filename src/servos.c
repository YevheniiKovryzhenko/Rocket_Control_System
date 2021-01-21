/**
 * @file servos.c
 */

#include <servos.h>


int servos_init(void)
{

    // initialize PRU
    if (rc_servo_init()) return -1;


    sstate.initialized = 1;
    return 0;
}

/*
* This function should be used anytime 
* the servos need to be returned to 
* their nominal (safe) positions
*/
static int __send_motor_nom_pulse(void)
{
    for (int i = 0; i < RC_SERVO_CH_MAX; i++) {
        sstate.ch[i] = 0; //have to set to calibrated nominal values
    }

    return 0;
}

int servos_arm(void)
{
    if (sstate.arm_state == ARMED) {
        printf("WARNING: trying to arm when servos are already armed\n");
        return -1;
    }
    if (sstate.initialized != 1)
    {
        printf("Servos have not been initialized \n");
        return -1;
    }
    // need to set each of the servos to their nominal positions:
    __send_motor_nom_pulse();


    //enable power:
    rc_servo_power_rail_en(1);

    sstate.arm_state = ARMED; //set servos to armed and powered
    return 0;
}

int servos_disarm(void)
{
    // need to set each of the servos to their nominal positions:
    //__send_motor_nom_pulse(); //won't work, need extra time before power is killed

    //power-off servo rail:
    rc_servo_power_rail_en(0);
    return 0;
}

int servos_march(int i, double* mot)
{
    //check if armed
    if (sstate.arm_state != ARMED)
    {
        printf("Servos have not been armed \n");
        return -1;
    }

    // need to do mapping between [0 1] and servo signal in us


    //send servo signals using [-1.5 1.5] normalized values
    if (rc_servo_send_pulse_normalized(i, mot[i]) == -1) return -1;

    //send servo signals using Pulse Width in microseconds
    //if (rc_servo_send_pulse_us(i, m[i]) == -1) return -1;

    return 0;
}

/*
static int running;
typedef enum test_mode_t{
        DISABLED,
        NORM,
        MICROSECONDS,
        SWEEP,
        RADIO
}test_mode_t;
static void __print_usage(void)
{
        printf("\n");
        printf(" Options\n");
        printf(" -c {channel}   Specify one channel from 1-8.\n");
        printf("                Otherwise all channels will be driven equally\n");
        printf(" -f {hz}        Specify pulse frequency, otherwise 50hz is used\n");
        printf(" -p {position}  Drive servo to a position between -1.5 & 1.5\n");
        printf(" -w {width_us}  Send pulse width in microseconds (us)\n");
        printf(" -s {limit}     Sweep servo back/forth between +- limit\n");
        printf("                Limit can be between 0 & 1.5\n");
        printf(" -r {ch}        Use DSM radio channel {ch} to control servo\n");
        printf(" -h             Print this help messege \n\n");
        printf("sample use to center servo channel 1:\n");
        printf("   rc_test_servo -c 1 -p 0.0\n\n");
}
// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

int main(int argc, char *argv[])
{
        double servo_pos=0;
        double sweep_limit=0;
        int width_us=0;
        int ch=0; // channel to test, 0 means all channels
        double direction = 1; // switches between 1 & -1 in sweep mode
        test_mode_t mode = DISABLED; //start mode disabled
        int frequency_hz = 50; // default 50hz frequency to send pulses
        int i, c;
        // parse arguments
        opterr = 0;
        while ((c = getopt(argc, argv, "c:f:p:w:s:h")) != -1){
                switch (c){
                // channel option
                case 'c':
                        ch = atoi(optarg);
                        if(ch<RC_SERVO_CH_MIN || ch>RC_SERVO_CH_MAX){
                                fprintf(stderr,"ERROR channel option must be between %d and %d\n", RC_SERVO_CH_MIN, RC_SERVO_CH_MAX);
                                return -1;
                        }
                        break;
                // pulse frequency option
                case 'f':
                        frequency_hz = atoi(optarg);
                        if(frequency_hz<1){
                                fprintf(stderr,"Frequency option must be >=1\n");
                                return -1;
                        }
                        break;
                // position option
                case 'p':
                        // make sure only one mode in requested
                        if(mode!=DISABLED){
                                __print_usage();
                                return -1;
                        }
                        servo_pos = atof(optarg);
                        if(servo_pos>1.5 || servo_pos<-1.5){
                                fprintf(stderr,"Servo position must be from -1.5 to 1.5\n");
                                return -1;
                        }
                        mode = NORM;
                        break;
                // width in microseconds option
                case 'w':
                        // make sure only one mode in requested
                        if(mode!=DISABLED){
                                __print_usage();
                                return -1;
                        }
                        width_us = atof(optarg);
                        if(width_us<10){
                                fprintf(stderr,"ERROR: Width in microseconds must be >10\n");
                                return -1;
                        }
                        mode = MICROSECONDS;
                        break;
                // sweep mode option
                case 's':
                        // make sure only one mode in requested
                        if(mode!=DISABLED){
                                __print_usage();
                                return -1;
                        }
                        sweep_limit = atof(optarg);
                        if(sweep_limit>1.5 || sweep_limit<-1.5){
                                fprintf(stderr,"ERROR: Sweep limit must be from -1.5 to 1.5\n");
                                return -1;
                        }
                        mode = SWEEP;
                        servo_pos = 0.0;
                        break;
                // help mode
                case 'h':
                        __print_usage();
                        return 0;
                default:
                        printf("\nInvalid Argument \n");
                        __print_usage();
                        return -1;
                }
        }
        // if the user didn't give enough arguments, exit
        if(mode==DISABLED){
                fprintf(stderr,"\nNot enough input arguments\n");
                __print_usage();
                return -1;
        }
        // set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running=1;
        // initialize PRU
        if(rc_servo_init()) return -1;
        // turn on power
        printf("Turning On 6V Servo Power Rail\n");
        rc_servo_power_rail_en(1);
        // print out what the program is doing
        printf("\n");
        if(ch==0) printf("Sending on all channels.\n");
        else      printf("Sending only to channel %d.\n", ch);
        switch(mode){
        case NORM:
                printf("Using rc_servo_send_pulse_normalized\n");
                printf("Normalized Signal: %f  Pulse Frequency: %d\n", servo_pos, frequency_hz);
                break;
        case MICROSECONDS:
                printf("Using rc_servo_send_pulse_us\n");
                printf("Pulse_width: %d  Pulse Frequency: %d\n", width_us, frequency_hz);
                break;
        case SWEEP:
                printf("Sweeping servos back/forth between +-%f\n", sweep_limit);
                printf("Pulse Frequency: %d\n", frequency_hz);
                break;
        default:
                // should never get here
                fprintf(stderr,"ERROR invalid mode enum\n");
                return -1;
        }
        // Main loop runs at frequency_hz
        while(running){
                switch(mode){
                case NORM:
                        if(rc_servo_send_pulse_normalized(ch,servo_pos)==-1) return -1;
                        break;
                case MICROSECONDS:
                        if(rc_servo_send_pulse_us(ch, width_us)==-1) return -1;
                        break;
                case SWEEP:
                        // increase or decrease position each loop
                        // scale with frequency
                        servo_pos += direction * sweep_limit / frequency_hz;
                        // reset pulse width at end of sweep
                        if(servo_pos>sweep_limit){
                                servo_pos = sweep_limit;
                                direction = -1;
                        }
                        else if(servo_pos < (-sweep_limit)){
                                servo_pos = -sweep_limit;
                                direction = 1;
                        }
                        // send result
                        if(rc_servo_send_pulse_normalized(ch,servo_pos)==-1) return -1;
                        break;
                default:
                        fprintf(stderr,"ERROR unhandled mode\n");
                        return -1;
                }
                // sleep roughly enough to maintain frequency_hz
                rc_usleep(1000000/frequency_hz);
        }
        rc_usleep(50000);
}
*/

int servos_cleanup(void)
{
    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    return 0;
}