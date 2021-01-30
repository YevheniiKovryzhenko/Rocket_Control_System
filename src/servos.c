/**
 * @file servos.c
 */

#include <servos.h>

servos_state_t sstate;

/*
This defines operating range of each servo mottor.
First column should the minimum position of each servo 
corresponding to the servo pulse signal in miliseconds 
(see rc_test_servos.c). Second column is the nominal/safe 
value for servos to return to when armed and before being 
disarmed. Signal should be in the range of [500 2500]us.
For details, see rc_servo_send_pulse_us().
*/
static double servos_lim[8][3] = \
{ {1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}};

/*
* This function should be used anytime 
* the servos need to be returned to 
* their nominal (safe) positions
*/
int __set_motor_nom_pulse(void)
{
    for (int i = 0; i < RC_SERVO_CH_MAX; i++) {
        sstate.m_us[i] = servos_lim[i][1]; //have to set to calibrated nominal values
    }

    return 0;
}

double __map_servo_signal_ms(double* m, double* servos_lim) {
    // sanity check
    if (m[0] > 1.0 || m[0] < 0.0) {
        printf("ERROR: desired thrust t must be between 0.0 & 1.0\n");
        return -1;
    }

    // return quickly for boundary conditions
    if (m[0] == 0.0) return servos_lim[0];
    if (m[0] == 1.0) return servos_lim[2];

    // Map [0 1] signal to servo pulse width
    return m[0] * (servos_lim[2] - servos_lim[0]) + servos_lim[0];

    fprintf(stderr, "ERROR: something in __map_servo_signal_ms went wrong\n");
    return -1;
}

int servos_init(void)
{
    sstate.arm_state = DISARMED;
    // initialize PRU
    if (rc_servo_init()) return -1;

    for (int i = 0; i < RC_SERVO_CH_MAX; i++) {
        sstate.m[i] = 0; //zero everything out just in case
    }
    __set_motor_nom_pulse();

    sstate.initialized = 1;
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
    __set_motor_nom_pulse();


    //enable power:
    rc_servo_power_rail_en(1);

    sstate.arm_state = ARMED; //set servos to armed and powered
    return 0;
}

int servos_disarm(void)
{
    // need to set each of the servos to their nominal positions:
    __set_motor_nom_pulse(); //won't work, need extra time before power is killed

    //send servo signals using Pulse Width in microseconds
    for (int i = 0; i < RC_SERVO_CH_MAX; i++) {
        if (rc_servo_send_pulse_us(i, sstate.m_us[i]) == -1) return -1;
    }

    //power-off servo rail:
    rc_servo_power_rail_en(0);
    return 0;
}

int servos_march(int i, double* mot)
{
    // need to do mapping between [0 1] and servo signal in us
    sstate.m_us[i] = __map_servo_signal_ms(mot, &servos_lim[i][3]);

    //send servo signals using [-1.5 1.5] normalized values
    //if (rc_servo_send_pulse_normalized(i, mot) == -1) return -1;

    //send servo signals using Pulse Width in microseconds
    if (rc_servo_send_pulse_us(i, sstate.m_us[i]) == -1) return -1;

    return 0;
}





int servos_cleanup(void)
{
    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    return 0;
}