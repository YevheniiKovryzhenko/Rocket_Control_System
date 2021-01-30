/**
 * <setpoint_manager.h>
 *
 * @brief      Setpoint manager runs at the same rate as the feedback controller
 *             and is the interface between the user inputs (input manager) and
 *             the feedback controller setpoint. currently it contains very
 *             simply logic and runs very quickly which is why it's okay to run
 *             in the feedback ISR right before the feedback controller. In the
 *             future this is where go-home and other higher level autonomy will
 *             live.
 *
 *             This serves to allow the feedback controller to be as simple and
 *             clean as possible by putting all high-level manipulation of the
 *             setpoints here. Then feedback-controller only needs to march the
 *             filters and zero them out when arming or enabling controllers
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

#include <rcs_defs.h>
#include <stdint.h> // for uint64_t
/**
 * Setpoint for the feedback controllers. This is written by setpoint_manager
 * and primarily read in by fly_controller. May also be read by printf_manager
 * and log_manager for telemetry
 */
typedef struct setpoint_t{

	/** @name general */
	///< @{
	int initialized;	///< set to 1 once setpoint manager has initialized
	///< @}
	uint64_t init_time;	///< time of activation 

	/** @name attitude setpoint */
	///< @{
	int en_alt_ctrl;	///< enable the altitude controller
	int en_r_ctrl;		///< enable the roll controller
	int en_py_ctrl;		///< enable the pitch yaw controllers
	double roll;		///< roll angle (rad)
	double pitch;		///< pitch angle (rad)
	double yaw;			///< yaw angle (rad)
	double X;
	double Y;
	double Z;
	double X_dot;
	double Y_dot;
	double Z_dot;
	double alt;			///< target altitude (limited by XYZ_MAX_ERROR)
	///< @}

	/** @name altitude */
	///< @{
	int en_X_ctrl;		///< enable altitude feedback.
	///< @}
} setpoint_t;

extern setpoint_t setpoint;

/**
 * @brief      Initializes the setpoint manager.
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_init(void);

/**
 * @brief      updates the setpoint manager, call this before feedback loop
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_update(void);

/**
 * @brief      cleans up the setpoint manager, not really necessary but here for
 *             completeness
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup(void);


#endif // SETPOINT_MANAGER_H
