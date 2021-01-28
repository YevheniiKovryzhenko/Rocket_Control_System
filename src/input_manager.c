/**
 * @file input_manager.c
 */

#include <input_manager.h>
#include <stdio.h>

user_input_t user_input; // extern variable in input_manager.h

int input_manager_init()
{
	user_input.requested_arm_mode = DISARMED;
	user_input.flight_mode = IDLE;		///< this is the user commanded flight_mode.
	user_input.input_active = 0;		///< nonzero indicates some user control is coming in

	//need to check serial connection and incoming data from other systems


	user_input.initialized = 1;
	return 0;
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