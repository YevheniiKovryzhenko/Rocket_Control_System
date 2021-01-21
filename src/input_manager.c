/**
 * @file input_manager.c
 */

#include <input_manager.h>

int input_manager_init()
{
	user_input.requested_arm_mode = DISARMED;
	user_input.flight_mode = IDLE;		///< this is the user commanded flight_mode.
	user_input.input_active = 0;		///< nonzero indicates some user control is coming in

	//need to check serial connection and incoming data from other systems


	user_input.initialized = 1; 
	return 0;
}