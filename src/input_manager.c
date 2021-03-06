/**
 * @file input_manager.c
 */

#include <input_manager.h>
#include <stdio.h>
#include <fallback_packet.h>
#include <state_estimator.h>

#include <rc/start_stop.h>
#include <rc/pthread.h>
#include <rc/time.h>
//#include <rc/dsm.h>
//#include <rc/math/other.h>

#include <rcs_defs.h>
#include <thread_defs.h>
//#include <setpoint_manager.h>

user_input_t user_input; // extern variable in input_manager.h
fallback_packet_t fallback;

static pthread_t input_manager_thread;

//This function will pick and choose which source of information to use (should only be called withing state_estimator.c)
int pick_data_source()
{
	if (user_input.initialized == 0) {
		fprintf(stderr, "ERROR in pick_data_source, input mannager was never initialized\n");
		return -1;
	}
	fallback = serialMsg;

	//always check these for external input: 
	user_input.use_external_state_estimation = fallback.use_external_state_estimation;

	//flight_status = fallback.flight_state;
	//if (fallback.flight_state == TEST) flight_status = fallback.flight_state; //get the flightstate
	//if (fallback.flight_state == STANDBY) flight_status = fallback.flight_state;
	
	user_input.requested_arm_mode = fallback.armed_state;
	if (user_input.use_external_state_estimation) //choose transmitted values, computed externally
	{
		//assume a single source of information for now

		//owerwrite the estimated state on the board with the external data:
		
		//state_estimate.alt_bmp			= fallback.alt; //get the altitude
		//state_estimate.alt_bmp_vel		= fallback.alt_vel; //get vertial velocity
		//state_estimate.proj_ap			= fallback.proj_ap;
		//state_estimate.alt_bmp_accel	= fallback.alt_accel; //get vertical accel
		//state_estimate.roll				= fallback.roll;
		//state_estimate.pitch			= fallback.pitch;
		//state_estimate.yaw				= fallback.yaw;
		
		//flight_status = fallback.flight_state;

		if (fallback.flight_state > flight_status)
		{
			flight_status++;
		}

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

void* input_manager(void* ptr)
{
	user_input.initialized = 1;
	// wait for first packet
	while (rc_get_state() != EXITING) {
		if (user_input.input_active) break;
		rc_usleep(1000000 / INPUT_MANAGER_HZ);
	}




	while (rc_get_state() != EXITING) {
		// if the core got disarmed, wait for arming sequence
		if (user_input.requested_arm_mode != ARMED && fallback.armed_state == ARMED) {
			// user may have pressed the pause button or shut down while waiting
			// check before continuing
			if (rc_get_state() != RUNNING) continue;
			else {
				user_input.requested_arm_mode = ARMED;
				//printf("\n\nDSM ARM REQUEST\n\n");
			}
		}
	}
	return NULL;
}

int input_manager_init()
{
	user_input.initialized = 0;
	int i;

	user_input.requested_arm_mode = DISARMED;
	user_input.flight_mode = IDLE;		///< this is the user commanded flight_mode.
	user_input.input_active = 0;		///< nonzero indicates some user control is coming in
	user_input.use_external_state_estimation = 0;		///< always start with relying on BBB data
	//need to check serial connection and incoming data from other systems

	// start thread
	if (rc_pthread_create(&input_manager_thread, &input_manager, NULL,
		SCHED_FIFO, INPUT_MANAGER_PRI) == -1) {
		fprintf(stderr, "ERROR in input_manager_init, failed to start thread\n");
		return -1;
	}
	// wait for thread to start
	for (i = 0; i < 50; i++) {
		if (user_input.initialized) return 0;
		rc_usleep(50000);
	}
	fprintf(stderr, "ERROR in input_manager_init, timeout waiting for thread to start\n");
	return -1;

	//user_input.initialized = 1;
	return 0;
}

int input_manager_cleanup() 
{
	if (user_input.initialized == 0) {
		fprintf(stderr, "WARNING in input_manager_cleanup, was never initialized\n");
		return -1;
	}
	// wait for the thread to exit
	if (rc_pthread_timed_join(input_manager_thread, NULL, INPUT_MANAGER_TOUT) == 1) {
		fprintf(stderr, "WARNING: in input_manager_cleanup, thread join timeout\n");
		return -1;
	}
	return 0;
}