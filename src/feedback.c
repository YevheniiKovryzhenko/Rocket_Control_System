/**
 * @file feedback.c
 *
 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/time.h>

#include <feedback.h>
#include <rcs_defs.h>
#include <servos.h>
#include <thrust_map.h>
#include <mix.h>
#include <settings.h>
#include <state_estimator.h>
#include <xbee_packet_t.h>
#include <setpoint_manager.h>
#include <log_manager.h>

#define TWO_PI (M_PI*2.0)

feedback_state_t fstate; // extern variable in feedback.h

// keep original controller gains for scaling later
static double D_roll_gain_orig, D_pitch_gain_orig, D_yaw_gain_orig, D_X_gain_orig;

// filters
static rc_filter_t D_roll	= RC_FILTER_INITIALIZER;
static rc_filter_t D_pitch	= RC_FILTER_INITIALIZER;
static rc_filter_t D_yaw	= RC_FILTER_INITIALIZER;
static rc_filter_t D_X		= RC_FILTER_INITIALIZER;


static void __rpy_init(void)
{
	// get controllers from settings

	rc_filter_duplicate(&D_roll, settings.roll_controller);
	rc_filter_duplicate(&D_pitch, settings.pitch_controller);
	rc_filter_duplicate(&D_yaw, settings.yaw_controller);

#ifdef DEBUG
	printf("ROLL CONTROLLER:\n");
	rc_filter_print(D_roll);
	printf("PITCH CONTROLLER:\n");
	rc_filter_print(D_pitch);
	printf("YAW CONTROLLER:\n");
	rc_filter_print(D_yaw);
#endif

	// save original gains as we will scale these by battery voltage later
	D_roll_gain_orig	= D_roll.gain;
	D_pitch_gain_orig	= D_pitch.gain;
	D_yaw_gain_orig		= D_yaw.gain;


	// enable saturation. these limits will be changed late but we need to
	// enable now so that soft start can also be enabled
	rc_filter_enable_saturation(&D_roll, -MAX_ROLL_COMPONENT, MAX_ROLL_COMPONENT);
	rc_filter_enable_saturation(&D_pitch, -MAX_PITCH_COMPONENT, MAX_PITCH_COMPONENT);
	rc_filter_enable_saturation(&D_yaw, -MAX_YAW_COMPONENT, MAX_YAW_COMPONENT);
	// enable soft start
	rc_filter_enable_soft_start(&D_roll, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_pitch, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_yaw, SOFT_START_SECONDS);
}


int feedback_disarm(void)
{
	fstate.arm_state = DISARMED;
	// set LEDs
	rc_led_set(RC_LED_RED, 1);
	rc_led_set(RC_LED_GREEN, 0);
	return 0;
}

int feedback_arm(void)
{
	if (fstate.arm_state == ARMED) {
		printf("WARNING: trying to arm when controller is already armed\n");
		return -1;
	}
	// start a new log file every time controller is armed, this may take some
	// time so do it before touching anything else
	if (settings.enable_logging) log_manager_init();
	// get the current time
	fstate.arm_time_ns = rc_nanos_since_boot();
	// reset the index
	fstate.loop_index = 0;
	
	//static int last_en_alt_ctrl = 0; //make sure altitude control will go through initialization

	// zero out all filters
	rc_filter_reset(&D_roll);
	rc_filter_reset(&D_pitch);
	rc_filter_reset(&D_yaw);
	rc_filter_reset(&D_X);

	// prefill filters with current error
	//rc_filter_prefill_inputs(&D_roll, -state_estimate.roll);
	rc_filter_prefill_inputs(&D_pitch, -state_estimate.pitch);
	rc_filter_prefill_inputs(&D_yaw, -state_estimate.yaw);
	// set LEDs
	rc_led_set(RC_LED_RED, 0);
	rc_led_set(RC_LED_GREEN, 1);
	// last thing is to flag as armed
	fstate.arm_state = ARMED;
	return 0;
}



int feedback_init(void)
{

	__rpy_init();		// roll, pitch yaw feedback initializer

	rc_filter_duplicate(&D_X, settings.altitude_controller);


#ifdef DEBUG
	printf("ALTITUDE CONTROLLER:\n");
	rc_filter_print(D_X);
#endif

	D_X_gain_orig = D_X.gain;

	rc_filter_enable_saturation(&D_X, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_X, SOFT_START_SECONDS);


	// make sure everything is disarmed them start the ISR
	feedback_disarm();

	fstate.initialized = 1;

	return 0;
}

int feedback_march(void)
{
	int i;
	double tmp, min, max;
	double u[MAX_INPUTS], mot[MAX_ROTORS];
	log_entry_t new_log;

	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if (rc_get_state() != RUNNING && fstate.arm_state == ARMED) {
		feedback_disarm();
		printf("\n rc_state is somehow paused \n");
	}

	/* Can be used later to activate attitude control
	// check for attitude deviation:
	if (fabs(state_estimate.yaw) > TIP_ANGLE || fabs(state_estimate.pitch) > TIP_ANGLE) {
		//activate attitude control
	}
	*/
	
	// We are about to start marching the individual SISO controllers forward.
	// Start by zeroing out the motors signals then add from there.
	for (i = 0; i < MAX_ROTORS; i++) mot[i] = 0.0;
	for (i = 0; i < MAX_INPUTS; i++) u[i] = 0.0;
	
	/***************************************************************************
	* Altitude Controller
	* run only if enabled
	***************************************************************************/
	if (setpoint.en_X_ctrl)
	{
		mix_check_saturation(VEC_X, mot, &min, &max);
		if (max > MAX_X_COMPONENT)  max = MAX_X_COMPONENT;
		if (min < -MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		rc_filter_enable_saturation(&D_X, min, max);
		D_X.gain = D_X_gain_orig * settings.v_nominal / state_estimate.v_batt_lp; //updating the gains based on battery voltage
		u[VEC_X] = rc_filter_march(&D_X, setpoint.alt - state_estimate.proj_app); //this has to be the error between target and predicted value
		mix_add_input(u[VEC_X], VEC_X, mot);
	}

	/***************************************************************************
	* Roll Pitch Yaw controllers, only run if enabled
	***************************************************************************/
	if (setpoint.en_r_ctrl) {
		// Roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if (max > MAX_ROLL_COMPONENT)  max = MAX_ROLL_COMPONENT;
		if (min < -MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		rc_filter_enable_saturation(&D_roll, min, max);
		D_roll.gain = D_roll_gain_orig * settings.v_nominal / state_estimate.v_batt_lp;
		u[VEC_ROLL] = rc_filter_march(&D_roll, setpoint.roll - state_estimate.roll);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);
	}

	if (setpoint.en_py_ctrl) {
		// Pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if (max > MAX_PITCH_COMPONENT)  max = MAX_PITCH_COMPONENT;
		if (min < -MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		rc_filter_enable_saturation(&D_pitch, min, max);
		D_pitch.gain = D_pitch_gain_orig * settings.v_nominal / state_estimate.v_batt_lp;
		u[VEC_PITCH] = rc_filter_march(&D_pitch, setpoint.pitch - state_estimate.pitch);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// Yaw
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if (max > MAX_YAW_COMPONENT)  max = MAX_YAW_COMPONENT;
		if (min < -MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		rc_filter_enable_saturation(&D_yaw, min, max);
		D_yaw.gain = D_yaw_gain_orig * settings.v_nominal / state_estimate.v_batt_lp;
		u[VEC_YAW] = rc_filter_march(&D_yaw, setpoint.yaw - state_estimate.yaw);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}

	/***************************************************************************
	* Send Actuator signals immediately at the end of the control loop
	***************************************************************************/
	for (i = 0; i < settings.num_rotors; i++) {
		rc_saturate_double(&mot[i], 0.0, 1.0);
		fstate.m[i] = map_motor_signal(mot[i]);

		// final saturation just to take care of possible rounding errors
		// this should not change the values and is probably excessive
		rc_saturate_double(&fstate.m[i], 0.0, 1.0);

		// finally send mapped signal to servos:
		servos_march(i, &mot[i]);
	}

	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for (i = 0; i < MAX_INPUTS; i++) fstate.u[i] = u[i];
	// keep track of loops since arming
	fstate.loop_index++;
	// log us since arming, mostly for the log
	fstate.last_step_ns = rc_nanos_since_boot();

	return 0;
}


int feedback_cleanup(void)
{
	//__send_motor_stop_pulse();

	servos_disarm();

	return 0;
}
