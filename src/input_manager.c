/**
 * @file input_manager.c
 */

#include <input_manager.h>
#include <stdio.h>
#include <fallback_packet.h>
#include <state_estimator.h>

user_input_t user_input; // extern variable in input_manager.h
fallback_packet_t fallback;

int input_manager_init()
{
	user_input.requested_arm_mode				= DISARMED;
	user_input.flight_mode						= IDLE;		///< this is the user commanded flight_mode.
	user_input.input_active						= 0;		///< nonzero indicates some user control is coming in
	user_input.use_external_state_estimation	= 0;		///< always start with relying on BBB data
	//need to check serial connection and incoming data from other systems


	user_input.initialized = 1;
	return 0;
}
//This function will pick and choose which source of information to use
int pick_data_source()
{
	//always check these for external input:
	user_input.requested_arm_mode = fallback.armed_state; 
	user_input.use_external_state_estimation = fallback.use_external_state_estimation;

	if (user_input.use_external_state_estimation) //choose transmitted values, computed externally
	{
		//assume a single source of information for now

		//owerwrite the estimated state on the board with the external data:
		state_estimate.alt_bmp			= fallback.alt; //get the altitude
		state_estimate.alt_bmp_vel		= fallback.alt_vel; //get vertial velocity
		state_estimate.alt_bmp_accel	= fallback.alt_accel; //get vertical accel
		state_estimate.roll				= fallback.roll;
		state_estimate.pitch			= fallback.pitch;
		state_estimate.yaw				= fallback.yaw;
		
		flight_status = fallback.flight_state;

		return 0;
	}
	else 
	{
		//don't do anything, no need to overwrite the state
		return 0;
	}


	//Should never reach this line
	return -1;
}

int input_manager_cleanup() 
{
	if (user_input.initialized == 0) {
		fprintf(stderr, "WARNING in input_manager_cleanup, was never initialized\n");
		return -1;
	}
	// wait for the thread to exit
	/*
	if (rc_pthread_timed_join(input_manager_thread, NULL, INPUT_MANAGER_TOUT) == 1) {
		fprintf(stderr, "WARNING: in input_manager_cleanup, thread join timeout\n");
		return -1;
	}
	*/
	return 0;
}