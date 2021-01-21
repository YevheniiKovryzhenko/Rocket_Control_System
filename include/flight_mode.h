/**
 * <flight_mode.h>
 */

#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

/**
 * This is how the user interacts with the setpoint manager.
 */
typedef enum flight_mode_t{
	/**
	 * IDLE mode does no feedback at all and should be used whenever the vehicle
	 * does not require control (on the launch pad, after completing the mission)
	 * but is not flagged as DISARMED 
	 */
	IDLE,
	/**
	 * APP_CTRL mode does only altitude feedback control using predicted appogee
	 */
	 APP_CTRL,
	/**
	 * YP_TEST mode is only for TESTING pitch and yaw controllers and is set to fight any
	 * attitude deviation from vertical.
	 */
	 YP_TEST,
	 /**
	 * YP_STABILIZE_APP mode has appogee controler enabled and uses pitch and yaw controllers for 
	 * active stabilization
	 */
	 YP_STABILIZE_APP
} flight_mode_t;




#endif