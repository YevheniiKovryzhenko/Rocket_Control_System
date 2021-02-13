#ifndef FALLBACK_PACKET_H
#define FALLBACK_PACKET_H

#include <setpoint_manager.h>

/*
<fallback_packet.h>

 * This header contains a packet structure with
 * everything essential for the system to perform well which can
 * be sent from supporting systems in the network

*/

/* Important!
* Altitude is always up / down, check with mix.c for coordinate frame definition
* Make sure all coordinate frames are properly matched! If using NED here, 
* make sure all of the data is properly converted all the data sent to this system.
*/
typedef struct fallback_packet_t
{
    uint32_t time;						///< Unique id or time for synchronizing transmition
	arm_state_t armed_state;			///< Use this channel to send ARMED or DISARMED command
	int use_external_state_estimation;	///< Use data computed externally (1)
	flight_status_t flight_state;	///< Use this channel to send current flight status
	double roll;					///< Rotation about x of the body 
	double pitch;					///< Rotation about y of the body 
	double yaw;						///< Rotation about z of the body 
	double proj_app;				///< estimated projected appogee the vehicle will reach given current flight condition
	double alt;						///< altitude estimate
	double alt_vel;					///< vertical velocity estimate 
	double alt_accel;				///< vertical accel estimate 
   
} fallback_packet_t;

extern fallback_packet_t fallback;

#endif //FALLBACK