/**
 * @file printf_manager.c
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>

#include <rcs_defs.h>
#include <printf_manager.h>
#include <input_manager.h>
#include <setpoint_manager.h>
#include <feedback.h>
#include <state_estimator.h>
#include <thread_defs.h>
#include <settings.h>

//B:
#include <xbee_packet_t.h>
#include <signal.h>
#include <rc/encoder.h>


static pthread_t printf_manager_thread;
static int initialized = 0;

const char* const colours[] = {KYEL, KCYN, KGRN, KMAG};
const int num_colours = 4; // length of above array
int current_colour = 0;

/**
 * @brief      { function_description }
 *
 * @return     string with ascii colour code
 */
static const char* __next_colour()
{
	// if reached the end of the colour list, loop around
	if(current_colour>=(num_colours-1)){
		current_colour=0;
		return colours[num_colours-1];
	}
	// else increment counter and return
	current_colour++;
	return colours[current_colour-1];
}

static void __reset_colour()
{
	current_colour = 0;
}


static int __print_header()
{
	int i;

	printf("\n");
	__reset_colour();
	if(settings.printf_arm){
		printf("  arm   |");
	}
	if (settings.printf_battery) {
		printf("%s v_batt |v_jack|", __next_colour());
	}
	if(settings.printf_altitude){
		printf("%s alt(m) |altdot|", __next_colour());
	}
	if (settings.printf_proj_ap) {
		printf("%saltacc|AP(m)|", __next_colour());
	}
	if(settings.printf_rpy){
		printf("%s roll|pitch| yaw |", __next_colour());
	}
	if(settings.printf_setpoint){
		printf("%s  sp_a | sp_r| sp_p| sp_y|", __next_colour());
	}
	if(settings.printf_u){
		printf("%s U0X | U1Y | U2Z | U3r | U4p | U5y |", __next_colour());
	}
	if(settings.printf_motors){
	//	printf("%s", __next_colour());
		for(i=0;i<settings.num_rotors;i++){
	//		printf("  M%d |", i+1);
		}
	}

	if(settings.printf_xbee){
 		printf("%s x_xb | y_xb | z_xb | qx_xb | qy_xb | qz_xb | qw_xb |", __next_colour());
 	}

 	if(settings.printf_rev){
 		printf("%s rev1 | rev2 | rev3 | rev4 ", __next_colour());
 	}

	printf(KNRM);
	if(settings.printf_mode){
		printf("   MODE ");
	}
	if (settings.printf_status) {
		printf("   FLIGHT STATUS ");
	}
	if(settings.printf_counter){
		printf(" counter ");
	}
	printf("\n");
	fflush(stdout);
	return 0;
}


static void* __printf_manager_func(__attribute__ ((unused)) void* ptr)
{
	int i;
	initialized = 1;
	printf("\nRocket Control System is initialized.\n");
	printf("Waiting for the remote arming sequence...\n\n");

	// turn off linewrap to avoid runaway prints
	printf(WRAP_DISABLE);

	// print the header
	__print_header();

	//sleep so state_estimator can run first
	rc_usleep(100000);

	while(rc_get_state()!=EXITING){

		printf("\r");
		if(settings.printf_arm){
			if(fstate.arm_state==ARMED) {
				printf("%s ARMED %s |", KRED, KNRM);
			/*} else if (fstate.arm_state == MID_ARMING) {
				printf("%sSTARTING%s|", KWHT, KNRM);*/
			} else {
				printf("%sDISARMED%s|", KGRN, KNRM);
			}
		}
		__reset_colour();
		if (settings.printf_battery) {
			printf("%s%+5.2f |%+5.2f |", __next_colour(),\
					state_estimate.v_batt_lp, \
					state_estimate.v_batt_lp_jack);
		}
		if(settings.printf_altitude){
			printf("%s%+5.2f |%+5.2f |",	__next_colour(),\
						state_estimate.alt_bmp,\
						state_estimate.alt_bmp_vel);
		}
		if (settings.printf_proj_ap) {
			printf("%s%+5.2f |%+5.2f |", __next_colour(), \
				state_estimate.alt_bmp_accel,\
				state_estimate.proj_ap);
		}
		if(settings.printf_rpy){
			printf(KCYN);
			printf("%s%+5.2f|%+5.2f|%+5.2f|",
							__next_colour(),\
							state_estimate.roll,\
							state_estimate.pitch,\
							state_estimate.yaw);
							//state_estimate.continuous_yaw);
		}
		if(settings.printf_setpoint){
			printf("%s%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							__next_colour(),\
							setpoint.Z,\
							setpoint.roll,\
							setpoint.pitch,\
							setpoint.yaw);
		}
		if(settings.printf_u){
			printf("%s%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							__next_colour(),\
							fstate.u[0],\
							fstate.u[1],\
							fstate.u[2],\
							fstate.u[3],\
							fstate.u[4],\
							fstate.u[5]);
		}		
		if(settings.printf_motors){
		//	printf("%s",__next_colour());
			for(i=0;i<settings.num_rotors;i++){
		//		printf("%+5.2f|", fstate.m[i]);
			}
		}
		printf(KNRM);
		if(settings.printf_xbee){
 			printf("%s%+5.2f |%+5.2f |%+5.2f | %+5.2f | %+5.2f | %+5.2f | %+5.2f |",\
 							__next_colour(),\
 							xbeeMsg.x,\
 							xbeeMsg.y,\
 							xbeeMsg.z,\
 							xbeeMsg.qx,\
 							xbeeMsg.qy,\
 							xbeeMsg.qz,\
 							xbeeMsg.qw);
 		}
		// we are not using encoders
 		if(settings.printf_rev){
			for(i=0;i<4;i++){
				printf("%10d|", state_estimate.rev[i]);
			}
 		}

		if(settings.printf_mode){
			print_flight_mode(user_input.flight_mode);
		}
		if (settings.printf_status) {
			print_flight_status(flight_status);
		}
		if(settings.printf_counter){
			printf("%d ",state_estimate.counter);
		}
		fflush(stdout);
		rc_usleep(1000000/PRINTF_MANAGER_HZ);
	}

	// put linewrap back on
	printf(WRAP_ENABLE);

	return NULL;
}

int printf_init()
{
	if(rc_pthread_create(&printf_manager_thread, __printf_manager_func, NULL,
				SCHED_FIFO, PRINTF_MANAGER_PRI)==-1){
		fprintf(stderr,"ERROR in start_printf_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(50000);
	return 0;
}


int printf_cleanup()
{
	int ret = 0;
	if(initialized){
		// wait for the thread to exit
		ret = rc_pthread_timed_join(printf_manager_thread,NULL,PRINTF_MANAGER_TOUT);
		if(ret==1) fprintf(stderr,"WARNING: printf_manager_thread exit timeout\n");
		else if(ret==-1) fprintf(stderr,"ERROR: failed to join printf_manager thread\n");
	}
	initialized = 0;
	return ret;
}


int print_flight_mode(flight_mode_t mode){
	switch(mode){
	case IDLE:
		printf("%sIDLE%s",KYEL,KNRM);
		return 0;
	case AP_CTRL:
		printf("%sAP_CTRL%s",KCYN,KNRM);
		return 0;
	case YP_TEST:
		printf("%sYP_TEST%s",KCYN,KNRM);
		return 0;
	case YP_STABILIZE_AP:
		printf("%sYP_STABILIZE_AP%s",KCYN,KNRM);
		return 0;
	default:
		fprintf(stderr,"ERROR in print_flight_mode, unknown flight mode\n");
		return -1;
	}
}
int print_flight_status(flight_status_t status) {
	switch (status) {
	case WAIT:
		printf("%s| WAIT%s", KYEL, KNRM);
		return 0;
	case STANDBY:
		printf("%s| STANDBY%s", KCYN, KNRM);
		return 0;
	case POWERED_ASCENT:
		printf("%s| POWERED_ASCENT%s", KCYN, KNRM);
		return 0;
	case UNPOWERED_ASCENT:
		printf("%s| UNPOWERED_ASCENT%s", KCYN, KNRM);
		return 0;
	case DESCENT_TO_LAND:
		printf("%s| DESCENT_TO_LAND%s", KCYN, KNRM);
		return 0;
	case LANDED:
		printf("%s| LANDED%s", KCYN, KNRM);
		return 0;
	case TEST:
		printf("%s| TEST%s", KCYN, KNRM);
		return 0;
	default:
		fprintf(stderr, "ERROR in print_flight_mode, unknown flight mode\n");
		return -1;
	}
}