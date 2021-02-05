/**
* @file setpoint_manager.c
*
*
**/
#include <stdio.h>
#include <math.h>
#include <string.h> // for memset

#include <rc/time.h> // for nanos
#include <inttypes.h> // for PRIu64
#include <rc/start_stop.h>
#include <rc/math/quaternion.h>

#include <input_manager.h>
#include <servos.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <feedback.h>
#include <state_estimator.h>
#include <rcs_defs.h>
#include <flight_mode.h>

#define XYZ_MAX_ERROR	700.0 ///< meters.

setpoint_t setpoint; // extern variable in setpoint_manager.h
flight_status_t flight_status;
events_t events;

//This is a shortcut function for calculating time elapsed since ti
double __finddt_s(uint64_t ti) {
	double dt_s = (rc_nanos_since_boot() - ti) / (1e9);
	return dt_s;
}


void __update_app(void)
{
	/*
	This function should be called to update the altitude controller command.
	Note that we are trying to limit the error input to the feedback controllers,
	therefore we need to avoid values above a certain limit. Even if the target appogee is 
	fixed, controller will see error in the range of XYZ_MAX_ERROR.
	We would theoretically want to make controller not do anything if projected appogee
	is bellow the target since we don't have control over propulsion system, but this should 
	not be done within this funtion and be a dedicated flight mode (CRUISE or DESCENT)

	setpoint.alt is one of the inputs to the altitude controller in feedback.c 
	that directly effects motion allong body-fixed X-axis which is
	in the direction of flight of the rocket vehicle. See mix.h, mix.s, feedback.c for more
	details.
	*/

	// make sure setpoint doesn't go too far to avoid controllers going crazy
	if (state_estimate.proj_app > (settings.target_altitude_m + XYZ_MAX_ERROR)) {
		setpoint.alt = state_estimate.proj_app + XYZ_MAX_ERROR;//if above target altitude
	}
	else if (state_estimate.proj_app < (settings.target_altitude_m - XYZ_MAX_ERROR)) {
		setpoint.alt = state_estimate.proj_app - XYZ_MAX_ERROR; //if below target altitude
		return;
	}
	else {
		setpoint.alt = state_estimate.proj_app; //dont limit the error
	}
	
	return;
}

int setpoint_manager_init(void)
{
	if(setpoint.initialized){
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}
	memset(&setpoint,0,sizeof(setpoint_t));

	//Need these for __flight_status_update()
	flight_status		= WAIT;
	events.burnout_fl	= 0;
	events.ignition_fl	= 0;

	
	user_input.flight_mode	= IDLE;
	setpoint.init_time		= rc_nanos_since_boot();
	setpoint.initialized	= 1;
	return 0;
}

/* __flight_status_update
*
* this function updates the flight status
*
* returns> 0 if sucessfull -1 on failure
*/
int __flight_status_update(void)
{
	//Check flight status:
	if (fstate.arm_state == DISARMED)
	{
		flight_status = WAIT;

	}
	else //if armed - start checking for events
	{
		if (flight_status == WAIT)
		{
			//make sure evrything is armed and ready:
			if (user_input.requested_arm_mode == ARMED) {
				if (fstate.arm_state == DISARMED) feedback_arm();
				if (sstate.arm_state == DISARMED) servos_arm();
			}

			//This should happen once the system just got armed (on the launchpad)
			events.ground_alt = state_estimate.alt_bmp;

			flight_status = STANDBY;
		}
		else if (flight_status == STANDBY)
		{
			if (fabs(state_estimate.alt_bmp_accel) >= settings.event_launch_accel && fabs(state_estimate.alt_bmp - events.ground_alt) >= settings.event_launch_dh)
			{
				events.init_time = rc_nanos_since_boot();
				events.ignition_alt = state_estimate.alt_bmp;
				events.ignition_fl = 1;
				events.init_time = rc_nanos_since_boot();
				flight_status = MOTOR_IGNITION;

			}
			else
			{
				//reset flags if the ignition has not been confirmed
				events.ignition_fl = 0;

				flight_status = STANDBY;
			}
		}
		else if (flight_status == MOTOR_IGNITION && __finddt_s(events.init_time) >= settings.event_ignition_delay_s && events.ignition_fl)
		{
			//accept the fact the motor is burning now
			flight_status = POWERED_ASCENT;
		}
	}

	return 0;
}



int setpoint_manager_update(void)
{
	if(setpoint.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
		return -1;
	}

	if(user_input.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
		return -1;
	}

	if (__flight_status_update() != 0) {
		fprintf(stderr, "ERROR in __flight_status_update.\n");
		return -1;
	}

	//for testing:
	/*
	if (__finddt_s(setpoint.init_time) > 10 && __finddt_s(setpoint.init_time) < 20)
	{
		user_input.requested_arm_mode = ARMED;
	}
	if (__finddt_s(setpoint.init_time) > 20 && __finddt_s(setpoint.init_time) < 60)
	{
		//user_input.requested_arm_mode = DISARMED;
		user_input.flight_mode = YP_TEST;
	}
	
	if (__finddt_s(setpoint.init_time) > 60)
	{
		user_input.requested_arm_mode = DISARMED;
	}
	*/

	// if PAUSED or UNINITIALIZED, do nothing
	if(rc_get_state()!=RUNNING) return 0;

	// shutdown feedback and servos on kill switch
	if(user_input.requested_arm_mode == DISARMED){
		if(fstate.arm_state != DISARMED) feedback_disarm();
		if(sstate.arm_state != DISARMED) servos_disarm();
		return 0;
	}

	// finally, switch between flight modes and adjust setpoint properly
	switch(user_input.flight_mode){
	case IDLE:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 0;
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 0;

		setpoint.roll	= 0;
		setpoint.pitch	= 0;
		setpoint.yaw	= 0;
		setpoint.alt	= 0;
		
		break;
	case APP_CTRL:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 1;
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 0;

		setpoint.roll = 0;
		setpoint.pitch = 0;
		setpoint.yaw = 0;
		
		__update_app();
		break;

	case YP_TEST:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 0; 
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 1;

		setpoint.roll	= 0;
		setpoint.pitch	= 0;
		setpoint.yaw	= 0;

		break;

	case YP_STABILIZE_APP:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 1;
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 1;

		setpoint.roll	= 0;
		setpoint.pitch	= 0;
		setpoint.yaw	= 0;

		__update_app();
		//TODO: implement limits on attitude control
		break;

	default: // should never get here
		fprintf(stderr,"ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.flight_mode)

	// arm feedback and servos when requested
	if(user_input.requested_arm_mode == ARMED){
		if(fstate.arm_state == DISARMED) feedback_arm();
		if(sstate.arm_state == DISARMED) servos_arm();
	}


	return 0;
}


int setpoint_manager_cleanup(void)
{
	setpoint.initialized=0;
	return 0;
}
